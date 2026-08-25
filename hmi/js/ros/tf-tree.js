/**
 * Minimal 2D TF cache: latest transform per child frame, composed on lookup.
 *
 * Why the cockpit needs TF at all: `/demo/scan` is stamped in `lidar`, and the
 * costmap, plan and footprint are in `map`. Drawing the scan on the map without
 * the chain map -> odom -> base -> trunk -> lidar puts the beams in the wrong
 * place, which looks like a broken lidar rather than a broken viewer.
 *
 * Why not just use /demo/odom and assume map == odom: on the QUADRUPED path
 * that happens to be true (odom_tf publishes map->odom as identity — there is
 * no AMCL, see nav_quadruped.launch.py). On the diff-drive path AMCL owns that
 * edge and it is not identity. An assumption that holds for one of two
 * supported robots is the kind that fails silently on the other.
 *
 * Deliberate simplifications, and what each costs:
 *
 *   - 2D only (x, y, yaw). The panel is a floor plan; roll and pitch of a
 *     walking quadruped would only tilt the beams out of the plane.
 *   - LATEST sample only, no time interpolation and no stamp matching. A scan
 *     is drawn against the newest pose rather than the pose at its own stamp.
 *     At walking speed (~0,07 m/s measured) and 10 Hz that is under a
 *     centimetre. Do not reuse this class for anything that closes a loop.
 *   - static and dynamic transforms share one table. /tf_static is ACCUMULATED,
 *     never replaced: robot_state_publisher and odom_tf are two different
 *     latched publishers, so the second message does not contain the first
 *     one's frames. Replacing would silently drop base->trunk.
 */

/** Pure: quaternion -> yaw. */
export function yawOf(q) {
  if (!q) return 0;
  const { x = 0, y = 0, z = 0, w = 1 } = q;
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

/** Pure: compose b∘a, both {x, y, theta}. Applies `a` first, then `b`. */
export function compose(b, a) {
  const cos = Math.cos(b.theta);
  const sin = Math.sin(b.theta);
  return {
    x: b.x + cos * a.x - sin * a.y,
    y: b.y + sin * a.x + cos * a.y,
    theta: b.theta + a.theta,
  };
}

/** Pure: apply a 2D transform to a point. */
export function applyTransform(t, x, y) {
  const cos = Math.cos(t.theta);
  const sin = Math.sin(t.theta);
  return { x: t.x + cos * x - sin * y, y: t.y + sin * x + cos * y };
}

export const IDENTITY = Object.freeze({ x: 0, y: 0, theta: 0 });

/** Guard against a malformed tree turning lookup into an infinite walk. */
const MAX_CHAIN_DEPTH = 32;

export class TfTree {
  constructor() {
    /** child frame -> {parent, x, y, theta} */
    this._edges = new Map();
  }

  /**
   * Ingest a tf2_msgs/TFMessage. Works for both /tf and /tf_static: the
   * accumulate-never-replace behaviour is what makes two latched publishers
   * coexist.
   */
  update(message) {
    for (const transform of message?.transforms ?? []) {
      const child = transform.child_frame_id;
      const parent = transform.header?.frame_id;
      if (!child || !parent) continue;
      const translation = transform.transform?.translation ?? {};
      this._edges.set(stripLeadingSlash(child), {
        parent: stripLeadingSlash(parent),
        x: translation.x ?? 0,
        y: translation.y ?? 0,
        theta: yawOf(transform.transform?.rotation),
      });
    }
  }

  has(frame) {
    return this._edges.has(stripLeadingSlash(frame));
  }

  clear() {
    this._edges.clear();
  }

  /** Frames that have a parent edge. Used by the panel to report what is missing. */
  frames() {
    return [...this._edges.keys()];
  }

  /**
   * Transform that maps a point expressed in `source` into `target`.
   *
   * Only walks upward from `source`, so `target` must be an ancestor of
   * `source` — which is the only direction this cockpit needs (everything is
   * drawn in `map`, and `map` is the root). Returns null when the chain is
   * incomplete, so the caller can say "esperando TF" instead of drawing
   * geometry at the origin, which is the failure that looks like a bug in the
   * robot.
   */
  lookup(target, source) {
    const want = stripLeadingSlash(target);
    let frame = stripLeadingSlash(source);
    let result = { ...IDENTITY };

    for (let depth = 0; depth < MAX_CHAIN_DEPTH; depth += 1) {
      if (frame === want) return result;
      const edge = this._edges.get(frame);
      if (!edge) return null;
      // Walking child -> parent: the parent's transform is applied on the
      // outside of everything accumulated so far.
      result = compose({ x: edge.x, y: edge.y, theta: edge.theta }, result);
      frame = edge.parent;
    }
    return null;
  }
}

/**
 * ROS 1 style leading slashes still show up in hand-written frame ids and in
 * some bridges. Two spellings of the same frame break the chain in a way that
 * reads as a missing transform.
 */
function stripLeadingSlash(frame) {
  return frame.startsWith('/') ? frame.slice(1) : frame;
}

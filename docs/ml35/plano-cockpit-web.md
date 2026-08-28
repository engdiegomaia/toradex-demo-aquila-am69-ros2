# Cockpit web — approved plan and decision registration

** Date:** 24/08/2026**State:** Approved plan. **F1 completed on
24/ZZXQ005QXZZ/ZZX0006QXZZ** — evidence on `docs/results/cockpit-web-f1.md`. Next phase:
F3b. ** Replaces:** the "Next recommended methodology" section of ZZXQ008QXZZ, which
recommended Qt/`rviz_common` and was discarded — see Decision 3.

Whoever resumes this work reads ** this file** and then goes straight to the section
"Where to resume." The survey has already been done; it does not need to be remade.

---

## 1. The problem, diagnosed

The failure of the previous four attempts was not a framework. It was an axis.

Repair (`XReparentWindow`) client windows that already belong to Mutter only work when
the client implements XEmbed. The Gazebo came in because it's a window Simple Qt; RViz2
and `rqt_image_view` did not enter and remained under their own `mutter-x11-frames`
frames. This is the composer's correct behavior, not his fault.

The only way to overcome this axis was to shut down the GNOME, which is already
prohibited in the procedure (the attempt of `gnome-shell --replace` interrupted the
operator session).

* *No UI technology solves this because the obstacle is below UI.** Switching PyQt5 to
GTK, Qt6, wxWidgets or Electron does not change anything.

---

## 2. Decisions taken at 24/08/2026

### Decision 1 — Switch the axis from "window capture" to "data"

There are two stable axes, both avoiding WM:

| Axis | How | What's embedded |
| --- | --- | --- |
| * *A — data** | each panel renders from topics ROS 2 | nothing; the app draws |
| **B — pixels** | each app on an X server**own**, transmitted | a framebuffer |

* ==References== On the B axis the GNOME would never see the windows (Gazebo would be
the only client of its Xvfb, the absolute owner of that X tree), which is mature
technique — but was discarded in Decision 4.

### Decision 2 — Cockpit web (`rosbridge` + `web_video_server` + HTML/CSS/ZZX005QXZZ)

Four options evaluated:

| Option | Image Layout | Gazebo/Real Viz | Reuse in kiosk AM69 | Choose |
| --- | --- | --- | --- | --- |
| **1. Cockpit web** | exact (CSS grid) | no (axis A) | **Total** | ==References== |
| 2. Lichtblick/Foxglove | approximate, unmarked | no | no | discarded |
| 3. Qt/C++ + `rviz_common` | exact | RViz yes, Gazebo no | no | discarded |
| 4. Continue with Xlib | — | — | — | rejected |

Concrete reasons of 1, not preference:

- * * This is what the project itself has already specified.** `.ai/AGENTS.md` §5.7 defines
  `ROS 2 → rosbridge_server → WebSocket → Chromium kiosk`, and the tree of §4 already
  reserve `hmi/` and `docker/hmi/` (the latter exists and is ** empty**). It is not a
  new path — it is the spec which has not yet been executed.
- * *One deliverable, two destinations. The same bundle is the host cockpit today and
  HMI of M3 in AM69 after. The options 2 and 3 would be discarded on the way.
- The control bar with Toradex logo and the log panel** only exist** on 1.
- The option 3 C++ force (no binding Python of `rviz_common`), contradicts
  "Python by default" convention of CLAUDE.md, and would still leave the Gazebo
  unsolved.

Registered caveat: Foxglove Studio open-source has been discontinued in old format and
active fork is Lichtblick (BMW). **This was not verified in this session** and did not
support the disposal of 2 option — it fell by layout and by not serving the kiosk.

### Decision 3 — Discard the recommendation of the previous checkpoint

`cockpit-standalone-parcial.md` recommended incorporating `rviz_common::RenderPanel` +
`VisualizationManager`. Discarded: is mandatory C++, solves only one of the five panels,
and does not serve the target arm64 (RViz2 is OGRE ZZX0004QXZZ and cannot run on
ZZXQ005QXZZ — ZZXQ006QXZZ rule of CLAUDE.md). That document remains valid as historical
record of the failure; his final recommendation, no.

### Decision 4 — Go straight to data fallback without the B-axis spike

The plan provided for a phase **F2**: spike of 90 min with Gazebo in X display isolated
+ acceleration by `/dev/dri` + noVNC in `<iframe>`, with
`glxinfo` reporting interactive hardware renderer and ≥15 ZZX0002QXZZ.

* *The operator decided to skip F2 and go straight to the F3b** (way by data).

Cost explicitly accepted with this decision:

- the camera-free orbit of Gazebo is lost;
- The display tree** of RViz is lost (on/off runtime layers);
- win-to-meta click on map, CPU/ZZX0001QXZZ free on host (Gazebo passa a
  `gui:=false`), and a single transport path for all panels.

Consequence: **F2 and F3a no longer exist.** The remaining stages are ZZX0001QXZZ → F3b
→ ZZX0002QXZZ → ZZX0003QXZZ → F6, kept the original names to marry this record.

### Decision 5 — No build npm step; rosbridge client itself

`.ai/AGENTS.md` §5.7 asks for "minimum and stable dependence set". The bundle will be
HTML/CSS/JS in ZZXQ005QXZZ pure modules, served by ZZXQ00006QXZZ (multi-arch), **without
`npm install`, without bundler, without build step**. That eliminates the supply chain
npm in arm64, which is where it hurts.

Consequence: `roslibjs` It'll be sold. The rosbridge v2 protocol is JSON simple
(`subscribe`/ZZX0003QXZZ/`publish`/ZZXQ005QXZZ/`call_service`) and a minimum customer
with reconnection fits into ~200 testable lines.

Context: the npm registry range check was denied by sandbox** in this session, so the
availability of `roslib` in the record ** was not confirmed**. That reinforces the
decision, but it wasn't her cause.

### Decision 6 — Scene camera goes in the world, never on `go2_description`

The blue panel ("GAZEBO SIMU ZZX0002QXZZ") becomes a bridged world camera for ROS. It
goes on `ros2_ws/src/demo_simulation/worlds/*.sdf`, which are project files.

`go2_description` is **sold with byte-a-byte warranty supporting the license argument**
(see the README package and the header of `bridge_quadruped.yaml`). It cannot be edited
— nor to hang a chase camera on the trunk. If a camera that follows the robot is needed
later, the path is a separate SDF model moved by the service gz `set_pose` from
ZZXQ005QXZZ, **not** edit the sellable package.

Start: two static cameras (isometric and top), alternated on UI.

### Decision 7 — `twist_mux` resolves the arbitration debit

The previous checkpoint recorded that teleop and Nav2 can publish in the same
`/demo/cmd_vel` without arbitration. Solved with `ros-jazzy-twist-mux` (standard
package, available in both architectures), manual priority > Nav2, with timeout. Don't
write mux by hand.

---

## 3. Evidence verified in this session

Checked in fact, not supposed:

| Fact | How it was verified |
| --- | --- |
| `ros-jazzy-rosbridge-suite` 2.7.0 | `apt-cache policy`, amd64 |
| `ros-jazzy-rosbridge-server` 2.7.0 | `apt-cache policy`, amd64 |
| `ros-jazzy-web-video-server` 3.1.0 | `apt-cache policy`, amd64 |
| `ros-jazzy-twist-mux` 4.5.0 | `apt-cache policy`, amd64 |
| `ros-jazzy-foxglove-bridge` 3.4.1 | `apt-cache policy`, amd64 |
| The five exist in **arm64** | `packages.ros.org/ros2/ubuntu dists/noble main/binary-arm64/Packages.gz` |
| Host has Node v24.15.0 / npm 11.14.1 | `node --version`, `npm --version` — **will not be required in runtime** |
| Npm log range | **Not verified** (command denied by sandbox) |
| Go2 has front camera | `go2_description/xacro/gazebo.xacro:259`, `<sensor name="front_camera" type="camera">` |
| Camera gets to the contract | `bridge_quadruped.yaml` → `/demo/camera/image_raw`, QoS reliable (deliberated; best-effort lost the fragments 640x480 no link) |
| Goals go by action**, not topic | `scripts/nav_trial.py:131` uses `ActionClient(NavigateToPose, 'navigate_to_pose')` |
| `/demo/navigation/status` | is at the convention of CLAUDE.md and **no public node** — needs to be created at F4 |
| `docker/hmi/` | exists and is ** empty** |
| `hmi/` at root | ** does not exist** yet (reserved in `.ai/AGENTS.md` §4) |
| `viz` already mounts `../scripts:/cockpit:ro` | `docker/compose.host.yml`, service `viz` |
| All services use `network_mode: host` | `x-common` in `compose.host.yml` — ports ** not** are mapped, bidam straight into the host |
| Existing Host Services | `base`, `sim`, `nav`(profile learn), `perception`(profile learn), `viz`, ZZXQ005QXZZ(profile tools) |
| `sim.launch.py` accepts `gui:=` | `sim.launch.py:95`, default `true` |
| Cockpit tests | `tests/test_cockpit_desktop_safety.py`, 5 tests (names in §6, F5) |

---

## 4. Approved architecture

Everything below is **host x86**, except where marked. Nav2 and perception continue in
AM69 without change.

```text
  ┌─ host x86 ───────────────────────────────────┐    ┌─ Aquila AM69 ─┐
  │  sim (Gazebo, gui:=false)  ──DDS──┐          │    │  nav (Nav2)   │
  │  viz:                             ├ rosbridge┼DDS─┤  perception   │
  │    web_video_server               │   :9090  │    └───────────────┘
  │    demo_hmi (relay de metas + status)        │
  │  hmi: nginx  ── bundle estático              │
  └──────────────────────────────────────────────┘
                    │
              Chromium (host hoje, kiosk no AM69 depois — M3)
```

Suggested doors: rosbridge 9090, `web_video_server` ZZX0002QXZZ, nginx 8081. Like
`network_mode: host`, they bind directly — **must be configurable by ZZXQ005QXZZ** so as
not to collide with anything from the operator. No IP hard-coded (CLAUDE.md).

Mapping of the five panels of the reference image
(`docs/ml35/cockpit-division-view.png`, the layout visual contract):

| Panel | Source | Transport |
| --- | --- | --- |
| **GAZEBO SIMU ZZX0002QXZZ** (blue) | world scene camera SDF → `/demo/cockpit/scene/image_raw` | `<img>` MJPEG |
| **RVIZ** (green) | map, footprint, `/demo/scan`, global plan, target, detection | `<canvas>` 2D via rosbridge |
| **RVIZ CAMERA** (light pink) | `/demo/camera/image_raw` + `/demo/perception/detections` | `<img>` MJPEG + overlay |
| **LOGS DE MOVEMENT** (pink) | `/rosout`, `/demo/cmd_vel`, `/demo/odom`, Nav2 status | rosbridge WebSocket |
| **BARRA DE ZZX0002QXZZ** (gray) | teleop → `twist_mux` → `/demo/cmd_vel`; goals → action | rosbridge WebSocket |

---

## 5. What DOES NOT Change

Untouched: Nav2, perception, topic contract, `sim.launch.py`, `compose.module.yml`,
`go2_description` (sold) and all ML3.5 ZZXQ005QXZZ–ZZXQ006QXZZ.

The additions are in places that the tree already reserved: `hmi/`,
`ros2_ws/src/demo_hmi/`, `docker/hmi/Dockerfile` and new services in
`docker/compose.host.yml`.

---

## 6. Remaining phases with gates

### F1 — Cockpit skeleton and transport (low risk)

Services `rosbridge` and `web_video_server`; `hmi` (`nginx:alpine`, multi-arch); bundle
`hmi/` in ZZXQ005QXZZ modules with own rosbridge client (ZZXQ006QXZZ); grid CSS
reproducing the image; state of connection, obsolete data and reconnection (§ZZXQ008QXZZ
of ZZX0009QXZZ); host/door by configuration.

> * *Gate:** the five regions appear in DP-1 in the ratio of the image; the panel
> camera shows `/demo/camera/image_raw` live; topple rosbridge changes the
> visual status for "disconnected" and it reconnects itself.

* *CONCLUDED in 24/08/ZZX0002QXZZ, learning mode at workstation.** Gate served on all
three items, with screenshots and logs on `docs/results/cockpit-web-f1.md`.

What stood up: Services `cockpit` (rosbridge 2.7.0 + web video server ZZX0002QXZZ,
multi-arch own image by construction) and `hmi` (`nginx:alpine`) in `compose.host.yml`;
ZZXQ006QXZZ in `demo_bringup`; bundle in ZZXQ008QXZZ with own rosbridge client,
never/live/stale freshness tracking, camera panel and functional log panel; 52 bundle
tests under `node --test` and 7 structural guards under pytest.

Two things cost time and are stuck by test — see the evidence document:
`web_video_server` does not make `topic` percentile, and a `<img>` with MJPEG does not
fire `load` in Firefox.

### F3b — Blue and green panels by data

- **Blue:**two static cameras (isometric and top) in `worlds/*.sdf`
  (Decision 6), bridged to `/demo/cockpit/scene/image_raw`; Gazebo with `gui:=false`.
- ** Green:** navigation view 2D in `<canvas>` — map, footprint,
  `/demo/scan`, global plan, target, detections. Click on the map → meta.

> * * Gate:** the two panels update live with the robot walking; the click on
> green panel generates a target that Nav2 accepts.

### F4 — Manual control with referee (closes known debit)

Package `demo_hmi` (`ament_python`): teleop node signing rosbridge commands; `twist_mux`
(Decision 7); relay `/goal_pose` → action ZZXQ005QXZZ; publication of ZZXQ006QXZZ (which
does not exist today).

Preserved semantics of `scripts/cockpit_teleop.py` — deadman/watchdog of 400 ms, halted
in release/focus loss/EOF, three zeros in shutdown — and the gear table already defined:

| Action | Linear. x | angular. z |
| --- | ---: | ---: |
| front | +0,25 | 0 |
| reverse | −0,20 | 0 |
| left | 0 | +0,20 |
| right | 0 | −0,20 |

> * * Gate:** the robot** actually moves** by the buttons — this is the honest debt of
> previous checkpoint, which has never been tested; with Nav2 navigating, tap the
> Teleop takes control and drop returns; E-STOP for <200 ms.

### F5 — Remove the dead path

Remove `scripts/cockpit.py` (PyQt5/Xlib, 672 lines) and `scripts/cockpit_teleop.py`;
retrace `scripts/run_cockpit.sh` (lifecycle logic, `stop_all`, ZZXQ005QXZZ and
ZZXQ006QXZZ is good and takes advantage).

Destination of 5 `tests/test_cockpit_desktop_safety.py` tests:

| Test | Destination |
| --- | --- |
| `test_launcher_cannot_manage_the_desktop_session` | ** keeps** — the ban on touching the GNOME continues to apply |
| `test_manual_control_has_deadman_and_zero_paths` | **maintain**, reappointed to `demo_hmi` |
| `test_standalone_window_embeds_all_three_clients` | ** Replace** — turns panel composition test |
| `test_controller_is_a_normal_top_level_window` | **Remove** — no more Qt window |
| `test_window_selection_ignores_qt_helpers` | **Remove** — No more window selection X11 |

### F6 — Evidence and documentation

`docs/results/cockpit-web.md` with measured FPS, teleop latency and screenshots;
replaces `cockpit-standalone-parcial.md` as current state. Update `.ai/CLAUDE.md` "Where
we are," `.ai/changelog.md` and ZZXQ005QXZZ.

* Out of scope: ** Kiosk Chromium accelerated at AM69. This is M3 and can only be
validated on real hardware. Do not state acceleration of GPU in the module from this
work (rule 7 of CLAUDE.md).

---

## 7. Registered risks

| Risk | Prob. | Mitigation |
| --- | --- | --- |
| `/map` great satura o rosbridge em JSON | Medium | rosbridge PNG compression; if failed, map turns stream of `web_video_server` |
| Action support ROS 2 no rosbridge 2.x not confirmed | Medium | the relay in `demo_hmi` (F4) does not depend on it — it is flat B embedded |
| Static cameras do not fit the maze11 (11,6 × 11,6 m) | Medium | two alternate poses; adjust against the footprint measured in the world header |
| Panels without interactivity do not convince the demo | Medium | **cost already accepted in Decision 4** |
| Latency of Ethernet link HIL on teleop | Medium | measure; F5 of ML3.5 already shows 8 m bursting protocol — do not mix the two problems |
| Doors 8080/9090 colliding on host | Low | configurable by `.env`; `network_mode: host` does not isolate |

---

## 8. Open points to be confirmed in implementation

1. ~~`rosbridge_suite` 2.x displays actions ROS ZZX0003QXZZ (`send_action_goal`)?~~
   **RESPONDIDO no F1: yes.** ZZX0002QXZZ records `SendActionGoal`, `ActionFeedback`,
   ZZXQ005QXZZ and `AdvertiseAction` on startup. The relay of F4 can shrink — but
   confirm with a long real goal before deleting plan B: registering capacity is not
   delivering feedback.
2. PNG compression of rosbridge is enough for the `/map` of maze11?
   (still open; `png_compression: true` is already connected to `cockpit.launch.py`)
3. ~~`web_video_server` accepts QoS reliable no `/demo/camera/image_raw`?~ ~
   **RESPONDIDO no F1: yes, without reconfiguration** ZZX0002QXZZ MB in ZZX0004QXZZ s
   of ZZXQ005QXZZ and ZZXQ006QXZZ returning JPEG ZZXQ008QXZZx480.
4. Frame of the two static cameras against the footprint of 11,6 m.
5. **New:** bundle was checked in Firefox. The kiosk of M3 is Chromium;
   `<img>` cache save in `config.js` comes from literature and was not measured.

---

## 9. Where to resume

F1 and **F3b** are closed (24/ZZX0002QXZZ/2026). Evidence of F3b, including simulation
control, camera control, Toradex identity and image quality numbers:
**`docs/results/cockpit-web-f3b.md`**. Operating guide (wheel, panels, controls,
scenarios, traps): **`docs/guia-completo.md`** (Part II).

What exists today on the screen:

| Region | Source | Controls |
| --- | --- | --- |
| blue, scene | two static cameras of the world | iso/top; rotate, tilt, move, zoom, refocus |
| green, navigation | costmap, plan, laser, footprint, TF | click send meta; cancel meta |
| pink, logs | `/rosout` + `cmd_vel`XQ0002QXZZ | — |
| Light pink, camera | `/demo/camera/image_raw` | — |
| bar | link status | simulation play/pause/reset |

### Next step: F4 — manual control

It is the only region of the screen that still lies: the arrow buttons and the E-STOP
are designed, keyboard-enabled, and** off**, with the motif in the `title`. Today Nav2
is the only publisher in `/demo/cmd_vel`; a teleop button that would also publish there
would give two unarbitrated writers on the same topic, with the last to write winning
and neither of them knowing they lost. Closes with `twist_mux` (Decision 7), not with a
mux handwritten in the browser.

### Next: F2 — kiosk in module

Chromium in kiosk mode in Aquila, with acceleration of GPU, serving this same bundle.
Three things still unmeasured and only the module responds:

1. the bundle was checked in **Firefox**; the kiosk is Chromium, and the caveat of
   `<img>` cache in `config.js` comes from literature, not measurement;
2. MJPEG of the scene cameras to 1600x1200 crossing the Ethernet — the
   parameter to first download is the JPEG quality in `hmi/js/config.js`, which
   degrades smoothly, and not the sensor resolution, which displaces the frame in
   pixels;
3. simulation services They exist in `deploy` mode: without Gazebo, the
   Play/pause/reset buttons need to disappear or tell you why they are not worth.
   That hasn't been taken care of.

### Restrictions that do not change

- * The simulator never goes to the module. Gazebo is OGRE 2 and AM69 only exposes
  OpenGL ES 3.2 and Vulkan ZZX0002QXZZ (rule 1). "Controlling the cockpit simulation"
  is supported; "turning the simulation in the module" is not, and no amount of code
  in UI changes that.
- * * The browser does not speak Gazebo types.** The rosbridge mounts the request
  importing the interface package inside the cockpit container, which has no
  `ros_gz_interfaces` — and in `deploy` mode would not even make sense. The border is
  `std_srvs`; the translation lives on the `sim_control_relay`, next to the simulator.
- **`go2_description` is sold and does not touch**, nor to hang one
  Camera in the trunk.

Before writing anything, reread `.ai/AGENTS.md` §5.7 (requirements of HMI) and §4
(reserved tree), and `docs/results/cockpit-web-f3b.md` (measured traps).

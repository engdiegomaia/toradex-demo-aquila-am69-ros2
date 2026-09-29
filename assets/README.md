# Brand assets

Source identity files, kept here so the derivatives used by the cockpit are
reproducible. **Nothing in this directory is served**: what actually ships is
in `hmi/img/`, already processed.

## `US Logo_Reverse.jpg`

Toradex brand mark, reverse version (white ink on `#00508d` blue
background), provided by the brand team.

It is the source for `hmi/img/toradex.png`, which needs to be **white on
transparent** — the cockpit bar is `#00508c` blue, and an opaque rectangle
there would show up as a visible patch. The conversion was a chroma key over
the background blue, with two steps that are not optional:

1. **unpremultiplied alpha**, otherwise the mark's green dot (which touches
   the background) disappears along with the blue;
2. **crop to the bounding box** before resizing, so the mark does not float
   inside transparent margin within the bar.

Result: 720 x 244, RGBA.

## `hmi/img/ros.png`

Has no source file here — it came ready-made from
<https://www.ros.org/imgs/logo-white.png>, already white on transparent.
520 x 137, RGBA.

ROS mark usage follows Open Robotics' guidelines.

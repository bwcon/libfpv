# Integrating libfpv into a game

The full hook sequence is shown in `examples/standalone/main.cpp` (the
comment at the top lists every call in order). This file holds the details
that need more than a comment.

## Lens distortion

FPV cameras use wide fisheye lenses. libfpv gives the parameters to use and
the game draws the effect, as a post-process on the FPV view only (not the
chase or LOS cameras):

1. Render the scene from `frame.camera` into an offscreen texture, using
   `frame.camera.fov_y_degrees` as the vertical FOV.
2. Draw a full-screen pass that samples that texture with this shader
   (GLSL; port it to HLSL or anything else as needed):

```glsl
uniform sampler2D scene;
uniform vec2 k;       // frame.camera.distortion_k1, distortion_k2
uniform float zoom;   // frame.camera.distortion_zoom
uniform float aspect; // output width / height
in vec2 uv;           // 0..1 across the output
out vec4 o;
void main() {
  vec2 scale = vec2(aspect, 1.0) / (0.5 * sqrt(aspect * aspect + 1.0)); // r = 1 at the corners
  vec2 c = (uv - 0.5) * scale;
  float r2 = dot(c, c);
  vec2 suv = c * (1.0 + k.x * r2 + k.y * r2 * r2) / zoom / scale + 0.5;
  o = (any(lessThan(suv, vec2(0.0))) || any(greaterThan(suv, vec2(1.0))))
      ? vec4(0.0, 0.0, 0.0, 1.0) : texture(scene, suv);
}
```

3. Draw the OSD (`fpv_osd_draw_data`) and the overlay on top, undistorted.

Without shaders (for example fixed-function D3D9), draw the copied scene
through a grid of about 64x36 cells instead. Give each vertex at output
position `uv` the texture coordinate `suv` from the formula above. The warp
is smooth, so the result matches the shader to within a pixel.

The player turns the fisheye on or off and sets its strength in F8 > Camera >
"Fisheye lens". It is on by default. When it is on, libfpv picks `k1`, `k2`
and a `zoom` that keep the image's corners filled (no black borders), and
widens `fov_y_degrees` so the top and bottom edges of the distorted image
still show the FOV the player chose. This widening uses the camera's
`aspect_hint`, so the vertical FOV is only exact when the output has that
aspect. When the slider is at 0, the preset's own `camera.k1` / `k2` / `zoom`
are used (normally none). A game that skips the post-process still works, but
shows a slightly wider, undistorted view.

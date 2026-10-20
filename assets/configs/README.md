# Betaflight configurations

`diff all` outputs a build of `examples/standalone` can start with instead
of libfpv's defaults: configure with
`-DLIBFPV_STANDALONE_BF_CONFIG=<path>` and every drone's configuration
starts from the file (`fpv_set_default_betaflight_diff`): on its first
flight and after a reset in F8 > Drone. A player's later changes are kept.

- `hammerill.diff.txt`: the page at https://libfpv.hammerill.com (set by
  `.github/workflows/web.yml`).

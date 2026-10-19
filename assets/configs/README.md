# Betaflight configurations

`diff all` outputs a build of `examples/standalone` can start with instead
of libfpv's defaults: configure with
`-DLIBFPV_STANDALONE_BF_CONFIG=<path>` and the game imports the file
(`fpv_import_betaflight_diff`) the first time it runs with a data directory.
A player's later changes are kept.

- `hammerill.diff.txt`: the page at https://libfpv.hammerill.com (set by
  `.github/workflows/web.yml`).

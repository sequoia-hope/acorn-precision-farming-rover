// Copy to local_config.js (gitignored) and fill in to enable the
// Mapbox satellite base layer and the OpenDroneMap orthophoto layer.
// Both keys are optional.
window.ACORN_LOCAL_CONFIG = {
  mapboxToken: "pk.<your-mapbox-public-token>",
  odm: {
    url: "http://<odm-host>:8090",
    username: "<user>",
    password: "<password>",
    tilesPath: "<project-id>/tasks/<task-uuid>",
  },
};

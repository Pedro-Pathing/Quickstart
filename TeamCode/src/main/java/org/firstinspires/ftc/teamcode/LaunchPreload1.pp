{
  "startPoint": {
    "x": 56,
    "y": 8,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-56mh19qevta",
      "name": "scorePoseBlue",
      "endPoint": {
        "x": 68,
        "y": 76,
        "heading": "linear",
        "startDeg": 270,
        "endDeg": 315
      },
      "controlPoints": [],
      "color": "#7B9B79",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkhbkdf5-a5r9hb",
      "name": "intakeAlign1Blue",
      "endPoint": {
        "x": 68,
        "y": 84,
        "heading": "linear",
        "reverse": false,
        "startDeg": 315,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#7C8AC7",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkhcv41h-tp75wv",
      "name": "intake1Blue",
      "endPoint": {
        "x": 16,
        "y": 84,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#97ACAD",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 144,
          "y": 70
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 120,
          "y": 144
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 25,
          "y": 144
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 7,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-56mh19qevta"
    },
    {
      "kind": "wait",
      "id": "mkhbkbjz-fvw6n3",
      "name": "Wait",
      "durationMs": 100,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mkhbkdf5-a5r9hb"
    },
    {
      "kind": "wait",
      "id": "mkhcv2dr-9ki1se",
      "name": "Wait",
      "durationMs": 0,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mkhcv41h-tp75wv"
    },
    {
      "kind": "wait",
      "id": "mkhcym4t-h9dnb7",
      "name": "Wait",
      "durationMs": 0,
      "locked": false
    }
  ],
  "settings": {
    "xVelocity": 75,
    "yVelocity": 65,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 16,
    "rHeight": 16,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "decode.webp",
    "robotImage": "/robot.png",
    "theme": "auto",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 3,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false
  },
  "version": "1.2.1",
  "timestamp": "2026-01-16T21:12:54.909Z"
}
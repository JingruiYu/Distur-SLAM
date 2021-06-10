#include "config.h"

const float config::correction = 1.0; //1.199
const float config::birdviewRows = 384.0;
const float config::birdviewCols = 384.0;
const float config::pixel2meter = 0.03984*correction;
const float config::meter2pixel = 25.1/correction;
const float config::rear_axle_to_center = 1.393;
const float config::vehicle_length = 4.63;
const float config::vehicle_width = 1.901;

const float config::noiseX = 1.0;
const float config::noiseY = 1.0;
const float config::noiseYaw = 1.0;

bool config::useLineForRotation = true;
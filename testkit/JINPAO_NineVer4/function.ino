RPM Inverse_Kinematic(float vx, float vy, float wz) {
  RPM wheel_rpms;

  wheel_rpms.rpm_FL = (vx + vy + (lx + ly) * wz) * angular_to_rpm;
  wheel_rpms.rpm_FR = (vx - vy - (lx + ly) * wz) * angular_to_rpm;
  wheel_rpms.rpm_BL = (vx - vy + (lx + ly) * wz) * angular_to_rpm;
  wheel_rpms.rpm_BR = (vx + vy - (lx + ly) * wz) * angular_to_rpm;

  return wheel_rpms;
}

Velocity Forward_Kinematic(float rpm_FL, float rpm_FR, float rpm_BL, float rpm_BR) {
  float rpm_to_rad_per_sec = 2 * PI / 60;

  float omega_FL = rpm_FL * rpm_to_rad_per_sec;
  float omega_FR = rpm_FR * rpm_to_rad_per_sec;
  float omega_BL = rpm_BL * rpm_to_rad_per_sec;
  float omega_BR = rpm_BR * rpm_to_rad_per_sec;

  Velocity bv;
  bv.vx = (wheelDiameter / 2) / 4 * (omega_FL + omega_FR + omega_BL + omega_BR);
  bv.vy = (wheelDiameter / 2) / 4 * -(-omega_FL + omega_FR + omega_BL - omega_BR);
  bv.wz = (wheelDiameter / 2) / (4 * (lx + ly)) * (omega_FL - omega_FR + omega_BL - omega_BR);

  return bv;
}
 
Position FindPosition(float rpm_FL, float rpm_FR, float rpm_BL, float rpm_BR, float dt, Position current) {
  Velocity bv = Forward_Kinematic(rpm_FL, rpm_FR, rpm_BL, rpm_BR);

  current.x += bv.vx * dt;
  current.y += bv.vy * dt;
  current.theta += bv.wz * dt * (180.0 / M_PI);

  current.theta = fmod(current.theta, 360.0);
  if (current.theta < 0) {
    current.theta += 360.0;
  }

  return current;
}

TransformStep parseTransformStep(String data) {
  TransformStep step;
  int index1 = data.indexOf(',');
  int index2 = data.indexOf(',', index1 + 1);
  int index3 = data.indexOf(',', index2 + 1);

  if (index1 != -1 && index2 != -1 && index3 != -1) {
    step.vx = data.substring(0, index1).toFloat();
    step.vy = data.substring(index1 + 1, index2).toFloat();
    step.wz = data.substring(index2 + 1, index3).toFloat();
    step.duration = data.substring(index3 + 1).toInt();
  } else {
    step.vx = 0;
    step.vy = 0;
    step.wz = 0;
    step.duration = 0;
  }

  return step;
}

void parseTransformSteps(String data, TransformStep* steps, int& stepsCount) {
  int i = 0;
  int lastPos = 0;
  int newPos;

  while ((newPos = data.indexOf(';', lastPos)) != -1) {
    steps[i++] = parseTransformStep(data.substring(lastPos, newPos));
    lastPos = newPos + 1;
  }

  if (lastPos < data.length()) {
    steps[i++] = parseTransformStep(data.substring(lastPos));
  }

  stepsCount = i;
}

float strToFloat(String str) {
  float result = 0.0;
  float decimalMultiplier = 1.0;
  bool isNegative = false;
  bool decimalPart = false;
  int length = str.length();

  if (str[0] == '-') {
    isNegative = true;
    str = str.substring(1);
    length--;
  }

  for (int i = 0; i < length; i++) {
    char c = str[i];

    if (c == '.') {
      decimalPart = true;
      continue;
    }

    if (decimalPart) {
      decimalMultiplier *= 0.1;
      result += (c - '0') * decimalMultiplier;
    } else {
      result = result * 10.0 + (c - '0');
    }
  }

  if (isNegative) {
    result = -result;
  }

  return result;
}

void TestKinematic(float x, float y, float z) {
  RPM wheel_rpms = Inverse_Kinematic(x,y,z);

  Serial.print(wheel_rpms.rpm_FL);
  Serial.print(" ");
  Serial.print(wheel_rpms.rpm_FR);
  Serial.print(" ");
  Serial.print(wheel_rpms.rpm_BL);
  Serial.print(" ");
  Serial.print(wheel_rpms.rpm_BR);
  Serial.print(" ");

  Velocity bv = Forward_Kinematic(wheel_rpms.rpm_FL, wheel_rpms.rpm_FR, wheel_rpms.rpm_BL, wheel_rpms.rpm_BR);

  Serial.print(bv.vx);
  Serial.print(" ");
  Serial.print(bv.vy);
  Serial.print(" ");
  Serial.print(bv.wz);
  Serial.print(" ");

  FL.compute(wheel_rpms.rpm_FL, deltaT);
  FR.compute(wheel_rpms.rpm_FR, deltaT);
  BL.compute(wheel_rpms.rpm_BL, deltaT);
  BR.compute(wheel_rpms.rpm_BR, deltaT);
  Serial.println(" ");
}
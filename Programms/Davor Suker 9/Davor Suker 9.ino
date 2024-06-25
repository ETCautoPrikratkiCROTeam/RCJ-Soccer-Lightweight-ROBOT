int StarTimer;
void linija2(){
  readLineSensors();
  if(lineUp){
    StarTimer = millis();
    while(true){
      go(30, 180, rotationToMaintainHeading(headingToMaintain), 30);
      if(StarTimer + 700 < millis()){
        break;
      }
      if(lineRight || lineLeft || lineUpRight || lineUpLeft){
        break;
      }
    }
  }
  if(lineDown){
    StarTimer = millis();
    while(true){
      go(30, 0, rotationToMaintainHeading(headingToMaintain), 30);
      if(StarTimer + 700 < millis()){
        break;
      }
      if(lineRight || lineLeft || lineDownRight || lineDownLeft){
        break;
      }
    }
  }


  if(lineRight){
    StarTimer = millis();
    while(true){
      go(30, -90, rotationToMaintainHeading(headingToMaintain), 30);
      if(StarTimer + 300 < millis()){
        break;
      }
      if(lineUp || lineDown || lineLeft || lineUpLeft || lineUpRight || lineDownRight || lineDownLeft){
        break;
      }
    }
  }

  if(lineLeft){
    StarTimer = millis();
    while(true){
      go(30, 90, rotationToMaintainHeading(headingToMaintain), 30);
      if(StarTimer + 300 < millis()){
        break;
      }
      if(lineUp || lineDown || lineRight || lineUpLeft || lineUpRight || lineDownRight || lineDownLeft){
        break;
      }
    }
  }

  if(lineUpRight){
    StarTimer = millis();
    while(true){
      go(30, -135, rotationToMaintainHeading(headingToMaintain), 30);
      if(StarTimer + 300 < millis()){
        break;
      }
      if(lineUp || lineDown || lineLeft || lineUpLeft || lineRight || lineDownRight || lineDownLeft){
        break;
      }
    }
  }

  if(lineUpLeft){
    StarTimer = millis();
    while(true){
      go(30, 135, rotationToMaintainHeading(headingToMaintain), 30);
      if(StarTimer + 300 < millis()){
        break;
      }
      if(lineUp || lineDown || lineLeft || lineUpRight || lineRight || lineDownRight || lineDownLeft){
        break;
      }
    }
  }

  if(lineDownRight){
    StarTimer = millis();
    while(true){
      go(30, -45, rotationToMaintainHeading(headingToMaintain), 30);
      if(StarTimer + 300 < millis()){
        break;
      }
      if(lineUp || lineDown || lineLeft || lineUpRight || lineRight || lineUpLeft || lineDownLeft){
        break;
      }
    }
  }

  if(lineDownLeft){
    StarTimer = millis();
    while(true){
      go(30, 45, rotationToMaintainHeading(headingToMaintain), 30);
      if(StarTimer + 300 < millis()){
        break;
      }
      if(lineUp || lineDown || lineLeft || lineUpRight || lineRight || lineUpLeft || lineDownRight){
        break;
      }
    }
  }
}

void linija(){
  readLineSensors();
  if (lineUp) {
  go(0, 0, 0, 0);
  delay(50);
  for (int i = 0; i < VRATI; i++) {
    go(30, 180, rotationToMaintainHeading(headingToMaintain), 30);
     delay(1);
  }
  delay(20);  // Additional delay for stability
}

if (lineDown) {
  go(0, 0, 0, 0);
  delay(50);
  for (int i = 0; i < VRATI; i++) {
    go(30, 0, rotationToMaintainHeading(headingToMaintain), 30);
     delay(1);
  }
  delay(20);  // Additional delay for stability
}

if (lineLeft) {
  go(0, 0, 0, 0);
  delay(50);
  for (int i = 0; i < VRATI; i++) {
    go(30, 90, rotationToMaintainHeading(headingToMaintain), 30);
     delay(1);
  }
  delay(20);  // Additional delay for stability
}

if (lineRight) {
  go(0, 0, 0, 0);
  delay(20);
  for (int i = 0; i < VRATI; i++) {
    go(30, -90, rotationToMaintainHeading(headingToMaintain), 30);
      delay(1);
  }
  delay(20);  // Additional delay for stability
}

if (lineUpRight) {
  go(0, 0, 0, 0);
  delay(50);
  for (int i = 0; i < VRATI; i++) {
    go(30, -135, rotationToMaintainHeading(headingToMaintain), 30);
     delay(1);
  }
  delay(20);  // Additional delay for stability
}

if (lineUpLeft) {
  go(0, 0, 0, 0);
  delay(50);
  for (int i = 0; i < VRATI; i++) {
    go(30, 135, rotationToMaintainHeading(headingToMaintain), 30);
     delay(1);
  }
  delay(20);  // Additional delay for stability
}

if (lineDownRight) {
  go(0, 0, 0, 0);
  delay(50);
  for (int i = 0; i < VRATI; i++) {
    go(30, -45, rotationToMaintainHeading(headingToMaintain), 30);
     delay(1);
  }
  delay(20);  // Additional delay for stability
}

if (lineDownLeft) {
  go(0, 0, 0, 0);
  delay(50);
  for (int i = 0; i < VRATI; i++) {
    go(30, 45, rotationToMaintainHeading(headingToMaintain), 30);
      delay(1);
  }
  delay(20);  // Additional delay for stability
}
}
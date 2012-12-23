import controlP5.*;
import processing.serial.Serial;
import processing.opengl.*;

ControlP5 cp5;
 
Serial serialPort;
int serialPortIndex;
public static final int
  IDLE_STATE = 0,
  HEADER_STATE = 1,
  LENGTH_STATE = 2;
int serialRxState = IDLE_STATE;

byte[] serialRxBuf = new byte[64];
int serialRxi=0;
int serialRxlen=0;

byte[] serialTxBuf = new byte[64];
int serialTxi=0;

int serialRequest = 0;


int RC_MIN = 900;
int RC_MID = 1500;
int RC_MAX = 2100;
  
int VR_MIN = -128;
int VR_MID = 0;
int VR_MAX = 127;

int PID_MIN = 0;
int PID_MAX = 511;

/***************************************************************************************************************
* 
***************************************************************************************************************/

ListBox comList;
Button cmdConnect, cmdDisconnect, cmdRead, cmdWrite, cmdDefault;

Numberbox pidRollP, pidRollI, pidRollD, pidPitchP, pidPitchI, pidPitchD, pidYawP, pidYawI, pidYawD;
Slider stkRate, stkGain;
Slider oscThres;
Slider rxMidAil, rxMidEle, rxMidRud;

CheckBox swCheckbox;

Slider rxAil, rxEle, rxRud, rxAux;
Slider vrAil, vrEle, vrRud;

Slider sxAil, sxEle, sxRud;

cData gyRoll, gyPitch, gyYaw;
boolean drawgyRoll, drawgyPitch, drawgyYaw;


Button addCmdButton(String name, int x, int y) {
  return cp5.addButton("id_cmd" + name)
    .setLabel(name)
    .setPosition(x, y)
    .setSize(110,18)
    ;
}


Textlabel addTextLabel(String name, int x, int y, int c) {
  return cp5.addTextlabel("id_txt" + name)
    .setText(name)
    .setPosition(x, y)
    .setColorValue(c)
    ;
}

Slider addSlider(String prefix, String name, int x, int y, int low, int high) {
  return cp5.addSlider(prefix + name)
    .setLabel(name)
    .setPosition(x, y)
    .setSize(150, 18)
    .setRange(low, high)
    .setValue(low)
    .setDecimalPrecision(0)
    .lock()
    ;
}

Slider addCfgSlider(String name, int x, int y, int low, int high) {
  Slider s = addSlider("id_cfg", name, x, y, low, high);
  s.unlock();
  return s;
}

Slider addRxSlider(String name, int x, int y) {
  return addSlider("id_rx", name, x, y, RC_MIN, RC_MAX);
}

Slider addSxSlider(String name, int x, int y) {
  return addSlider("id_sx", name, x, y, RC_MIN, RC_MAX);
}

Slider addVrSlider(String name, int x, int y) {
  return addSlider("id_vr", name, x, y, VR_MIN, VR_MAX);
}

Numberbox addPIDBox(String name, int x, int y, int spacing) {
  return cp5.addNumberbox("id_pid" + name)
   .setPosition(x, y)
   .setSize(spacing-10, 18)
   .setRange(PID_MIN, PID_MAX)
   .setValue(PID_MIN)
   .setDirection(Controller.HORIZONTAL) // change the control direction to left/right
   .setDecimalPrecision(0)
   .setLabel("")
   .setLabelVisible(false)
   ;
}



void serialPortInit(int index) {
  println("serialPortInit " + index);
  serialPortIndex = index;
  for (int i=0; i<comList.getListBoxItems().length; i++)
    comList.getItem(i).setColorBackground(0);
  comList.getItem(index).setColorBackground(color(0,128,0));
}

void id_cmdConnect(int val){
  println("id_cmdConnect handler" + val);
  if (serialPort == null) {
    serialPort = new Serial(this, Serial.list()[serialPortIndex], 115200);
  }
}

void id_cmdDisconnect(int val) {
  println("id_cmdDisconnect handler" + val);
  serialPort.stop();
  serialPort = null;
}


void id_cmdDefault(int val) {
  println("id_cmdDefault handler" + val);
    sendGetGyro();
}

/***************************************************************************************************************
* 
***************************************************************************************************************/

void setup() {
//  size(800, 600, OPENGL);
  size(800, 600); // must be "first" line in setup()
  noStroke();
  background(color(32,32,32));
  frameRate(20); 
  smooth();

  cp5 = new ControlP5(this); // initialize the GUI controls
  cp5.setControlFont(createFont("Arial bold",15,false));

  int x, y;
  int dy = 20;


  x = 10; y = 20;

  comList = cp5.addListBox("id_serial")
          .setLabel("Serial")
          .setPosition(x, y)
          .setSize(120, 100)
          .setItemHeight(20)
          .setBarHeight(15)
          ;

  for (int i=0; i < Serial.list().length; i++) {
    comList.addItem(Serial.list()[i], i);
  }
  if (Serial.list().length == 1) {
    serialPortInit(0);
  }

  x = 10; y = 200;

  addTextLabel("COMMAND", x, y, color(255, 255, 0));
  cmdConnect = addCmdButton("Connect", x, y += dy);
  cmdDisconnect = addCmdButton("Disconnect", x, y += dy);
  cmdRead = addCmdButton("Read", x, y += dy);
  cmdWrite = addCmdButton("Write", x, y += dy);
  cmdDefault = addCmdButton("Default", x, y += dy);
  

  x = 150; y = 20;
  int sep = 50;
  addTextLabel("P", x + sep, y, color(255, 255, 0));
  addTextLabel("I", x + sep * 2, y, color(255, 255, 0));
  addTextLabel("D", x + sep * 3, y, color(255, 255, 0));
  
  addTextLabel("Roll", x, y += dy, color(255, 255, 0));
  pidRollP = addPIDBox("RollP", x + sep, y, sep);
  pidRollI = addPIDBox("RollI", x + sep * 2, y, sep);
  pidRollD = addPIDBox("RollD", x + sep * 3, y, sep);

  addTextLabel("Pitch", x, y += dy, color(255, 255, 0));
  pidPitchP = addPIDBox("PitchP", x + sep, y, sep);
  pidPitchI = addPIDBox("PitchI", x + sep * 2, y, sep);
  pidPitchD = addPIDBox("PitchD", x + sep * 3, y, sep);

  addTextLabel("Yaw", x, y += dy, color(255, 255, 0));
  pidYawP = addPIDBox("YawP", x + sep, y, sep);
  pidYawI = addPIDBox("YawI", x + sep * 2, y, sep);
  pidYawD = addPIDBox("YawD", x + sep * 3, y, sep);

  y += 40;
  addTextLabel("RX MID", x, y, color(255, 255, 0));
  rxMidAil = addCfgSlider("Ail ", x, y += dy, RC_MIN, RC_MAX);
  rxMidEle = addCfgSlider("Ele ", x, y += dy, RC_MIN, RC_MAX);
  rxMidRud = addCfgSlider("Rud ", x, y += dy, RC_MIN, RC_MAX);

  y += 40;
  addTextLabel("CFG", x, y, color(255, 255, 0));
  stkGain = addCfgSlider("Atten", x, y += dy, 0, 1023);
  stkRate = addCfgSlider("Rate", x, y += dy, 0, 1023);
  oscThres = addCfgSlider("Thres", x, y += dy, 0, 1023);





     
  x = 375; y = 20;    
  addTextLabel("RX", x, y, color(255, 255, 0));
  rxAil = addRxSlider("Ail", x, y += dy);
  rxEle = addRxSlider("Ele", x, y += dy);
  rxRud = addRxSlider("Rud", x, y += dy);
  rxAux = addRxSlider("Aux", x, y += dy);

  y += 40;
  addTextLabel("VR", x, y, color(255, 255, 0));
  vrAil = addVrSlider("Ail", x, y += dy);
  vrEle = addVrSlider("Ele", x, y += dy);
  vrRud = addVrSlider("Rud", x, y += dy);

  y += 40;
  addTextLabel("SWITCH", x, y, color(255, 255, 0));
  swCheckbox = cp5.addCheckBox("id_swCheckbox")
    .setPosition(x, y += dy)
    .setSize(12, 12)
    .setSpacingColumn(50)
    .setItemsPerRow(3)
    .addItem("Ail ", 0)
    .addItem("Ele ", 0)
    .addItem("Rud ", 0)
    ;
  swCheckbox.getItem(0).lock();
  swCheckbox.getItem(1).lock();
  swCheckbox.getItem(2).lock();

  x = 600; y = 20;
  addTextLabel("SERVO", x, y, color(255, 255, 0));
  sxAil = addSxSlider("Ail", x, y += dy);
  sxEle = addSxSlider("Ele", x, y += dy);
  sxRud = addSxSlider("Rud", x, y += dy);
  
  gyRoll = new cData(200);
  gyPitch = new cData(200);
  gyYaw = new cData(200);
  
}

/***************************************************************************************************************
* 
***************************************************************************************************************/



class cData {
  float[] value;
  int first, last, count, maxSize;
  
  cData(int _maxSize) {
    maxSize = _maxSize;
    value = new float[maxSize];
    first = last = count = 0;
  }
  
  void appendValue(float new_value) {
    value[last] = new_value;
    last = (last + 1) % maxSize;
    if (count < maxSize)
      count++;
    else
      first = (first + 1) % maxSize;
  }
      
  int getCount() {
    return count;
  }

  float getValue(int i) {
    return value[(first + i) % maxSize];
  }
  
  int getMaxSize() {
    return maxSize;
  }
}



int graphX = 100, graphY = 400, graphWidth = 600, graphHeight = 200;


void drawGraph(cData g, color c) {
  float y, y0=0, x;
  float graphdX = (float)graphWidth / 200;
 
  stroke(c);
  strokeWeight(1.5);
  for (int i=0; i<g.getCount(); i++) {
    y = map(g.getValue(i), -33000.0, +33000.0, graphY + graphHeight, graphY);
    if (i>0)
      line(graphX + i*graphdX, y0, graphX + (i+1)*graphdX, y); 
    y0 = y;
  }
}



void drawGraphs() {
  fill(0);
  stroke(0);
  rect(graphX, graphY, graphWidth, graphHeight);
  
  drawGraph(gyRoll, color(255, 0, 0));  
  drawGraph(gyPitch, color(0, 255, 0));  
  drawGraph(gyYaw, color(0, 0, 255));  

}


void draw() {
  //println("Draw");
//  fill(123 % 100);
//  rect(0,0,500,500);
//  nb.setValue(nb.getValue()+1);
//  rxAil.setValue(rxAil.getValue()+1).show();

  drawGraphs();
}

void keyPressed() {
  println("keyPressed()");
  //cp5.getController("Ail").setValue(0);
  swCheckbox.activate(2);
  
  println(cp5.getAll());  
  gyRoll.appendValue(random(100));
}

void controlEvent(ControlEvent theEvent) {
  println("controlEvent " + theEvent.name());
  if (theEvent.name() == "id_swCheckbox")
    println(swCheckbox.getState(0));  

  if (theEvent.isGroup() && theEvent.name() == "id_serial")
    serialPortInit((int)theEvent.group().value());
    
}


/***************************************************************************************************************
* 
***************************************************************************************************************/

void serialSendBuf(byte[] buf, int len)
{
  byte checksum;
  int i;
  
  if (serialRequest > 0) {
    println("serialRequest > 0");
  }

  checksum = (byte)(len+1);
  for (i=0; i<len; i++)
    checksum += buf[i];
  checksum ^= 0xff;
  checksum += 1;
  
  serialPort.write('$');
  serialPort.write((byte)(len+1));
  for (i=0; i<len; i++)
    serialPort.write(buf[i]);
  serialPort.write((byte)checksum);
    
  /*
  print("$ " + hex((byte)(len+1)) + " ");
  for (i=0; i<len; i++)
    print(hex(buf[i]) + " ");
  println(hex(checksum));
  */
  
  serialRequest++;
}


void put8(int v)
{
  serialTxBuf[serialTxi++] = (byte) v;
}

void put16(int v)
{
  put8(v & 0xff);
  put8((v >> 8) & 0xff);
}

void put32(int v)
{
  put16(v & 0xffff);
  put16((v >> 16) & 0xffff);
}


int get_unsigned8()
{
  return serialRxBuf[serialRxi++] & 0xff;
}

int get_signed16() {
  return (serialRxBuf[serialRxi++] & 0xff) | (serialRxBuf[serialRxi++] << 8);
}

int get_signed32() {
   return (serialRxBuf[serialRxi++] & 0xff) | 
     ((serialRxBuf[serialRxi++] & 0xff) << 8) |
     ((serialRxBuf[serialRxi++] & 0xff) << 16) |
     (serialRxBuf[serialRxi++] << 24);
}


void handleResponse()
{
  println("handleResponse");
  serialRxi = 0;
  switch (get_unsigned8()) {
  case 101:
    int gRoll = get_signed16();  
    int gPitch = get_signed16();  
    int gYaw = get_signed16();  
    
    gyRoll.appendValue(gRoll);
    gyPitch.appendValue(gPitch);
    gyYaw.appendValue(gYaw);
    break;
  }
  

  // decide next request message  
  sendGetGyro();
}

void serialEvent(Serial serialPort)
{
  while (serialPort.available() > 0) { 
    byte ch = (byte) serialPort.read();
    //println(hex(ch));
    switch(serialRxState) {
    case IDLE_STATE:
      if (ch == (byte)'$') 
        serialRxState = HEADER_STATE;
      break;
    case HEADER_STATE:
      if (2 <= ch && ch <= serialRxBuf.length) {
        serialRxlen = ch;
        serialRxi = 0;
        serialRxState = LENGTH_STATE;
      } else {
        serialRxState = IDLE_STATE;
      }
      break;
    case LENGTH_STATE:
      serialRxBuf[serialRxi++] = ch;
      if (--serialRxlen == 0) {
        byte checksum = (byte)serialRxi;
        for (int i=0; i<serialRxi; i++)
          checksum += serialRxBuf[i];
        if (checksum == 0) {
          serialRequest--;
          handleResponse();
        } else {
          println("bad checksum");          
        }   
        serialRxState = IDLE_STATE;
      }
      break;    
    }
    
    if (serialRxState == IDLE_STATE)
      break;
  }
}

void sendGetGyro() {
  serialTxi = 0;
  put8(100); 
  serialSendBuf(serialTxBuf, serialTxi);
}



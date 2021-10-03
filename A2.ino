#define inputCLK1 13
#define inputDT1 12
#define inputCLK2 8
#define inputDT2 7
  int clk1;
  int dt1;
  int clk2;
  int dt2;
  long ctr = 0;
  long ctr1 = 0;
  int oldpos1= 0;
  int oldpos2 = 0;
  long oldtime1 = 0;
  int oldtime2 = 0;
  double vel = 0;

void setup() {
  // put your setup code here, to run once:
  //input signals assignments
  clk1 = digitalRead(inputCLK1);
  dt1 = digitalRead(inputDT1);
  clk2 = digitalRead(inputCLK2);
  dt2 = digitalRead(inputDT2);
  //set up inputs
  pinMode(inputCLK1, INPUT);
  pinMode(inputDT1, INPUT);
  pinMode(inputCLK2, INPUT);
  pinMode(inputDT2, INPUT);
  //set pins as Vcc
  digitalWrite(inputCLK1, HIGH);
  digitalWrite(inputDT1, HIGH);
  digitalWrite(inputCLK2, HIGH);
  digitalWrite(inputDT2, HIGH);
  //read input signals
  Serial.begin(9600);
 
}

double pos(){
  int time1 = newtime;
  int currentCLK = digitalRead(inputCLK1);
  int currentDT = digitalRead(inputDT1);
  if (currentCLK != clk1){
    if(currentDT != currentCLK){
     ctr++; 
    }
    else{
      ctr--;
    }
  }
 double velo = (ctr-oldpos1)/(time1-oldtime1);
 oldpos1 = ctr;
 oldtime1 = time1;
 return velo;
}
double pos1(){
  int time2 = newtime;
  int currentCLK1 = digitalRead(inputCLK2);
  int currentDT1 = digitalRead(inputDT2);
  if (currentCLK1 != clk1){
    if(currentDT1 != currentCLK1){
     ctr1++; 
    }
    else{
      ctr1--;
    }
  }
 double vel1 = (ctr1-oldpos2)/(time2-oldtime2); // calculate velocity distance/time
  oldpos2 = ctr1;
  oldtime2 = time2;
  return vel1;
}
void loop() {
  long time1 = millis();
  long currentCLK = digitalRead(inputCLK1);
  long currentDT = digitalRead(inputDT1);
  if (currentCLK != clk1){
    if(currentDT != currentCLK){
     ctr++; 
    }
    else{
      ctr--;
    }
  }
 ctr=(2*3.1415*ctr)/3200; // covert counts to radians
 vel = (ctr-oldpos1)/((time1-oldtime1)*0.001); // multiply by 0.001 for millis 
 oldpos1 = ctr;
 oldtime1 = time1;
}

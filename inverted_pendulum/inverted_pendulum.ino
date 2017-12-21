#define vr_pin A0

#define dir_pin_1 7
#define dir_pin_2 9
#define speed_ctrl_pin 3

int print_prescaler = 50;

float pendulum_angle = 0.0f;

float integrator = 0.0f;

void setup()
{
  Serial.begin(9600);
  
  pinMode(INPUT, vr_pin);
  pinMode(OUTPUT, dir_pin_1);
  pinMode(OUTPUT, dir_pin_2);
  pinMode(OUTPUT, speed_ctrl_pin);
}

void read_angle()
{
  int vr = analogRead(vr_pin);
  pendulum_angle = (float)vr / 1023.0f * 270.0f;
  
  pendulum_angle -= 82.4; //Tune me
}

void rotate_left()
{
  digitalWrite(dir_pin_1, HIGH);
  digitalWrite(dir_pin_2, LOW);
}

void rotate_right()
{
  digitalWrite(dir_pin_1, LOW);
  digitalWrite(dir_pin_2, HIGH);
}

void set_speed(int _speed)
{
  analogWrite(speed_ctrl_pin, _speed);
}

float bound(float val, float _min, float _max)
{
  if(val > _max) {
      return _max;
  } else if(val < _min) {
      return _min;
  } else {
      return val;      
  }
}

void feedback_control()
{
  read_angle();
  
  //Tune these parameters!
  float KP = 10.0f;
  float KI = 0.007f;

  float error = -pendulum_angle;
  
  float P = error * KP;
  float I = 0.0f;
  float D = 0.0f;

  integrator += error * KI;
  
  P = bound(P, -255.0f, 255.0f);
  I = bound(integrator, -255.0f, 255.0f);
  //D = bound(D, -255.0f, 255.0f);
  
  float PID = P + I + D;
  PID = bound(PID, -255.0f, 255.0f);
  
  if(PID > 0.0f) {
    rotate_right();
  } else {
    rotate_left();
  }

  int output = (int)abs(PID);

  set_speed(output);
  
  print_prescaler--;
  if(print_prescaler == 0) {
    print_prescaler = 50;
    #if 0
    Serial.print("speed:");
    Serial.print(output);
    Serial.print(",angle:");
    #endif
    Serial.println(pendulum_angle);
    //Serial.println(I);
  }
}

void loop()
{  
  feedback_control();
}

// Incluimos los archivos de encabezado necesarios para usar ROS y crear mensajes de tipo std_msgs/Float32
#include <ros.h>
#include <std_msgs/Float32.h>

// Incluimos la librería de los sensores
#include <MQUnifiedsensor.h>

// Definimos las variables de los sensores
#define board "Arduino UNO"
#define Voltage_Resolution 5
#define pin_mq135 A0
#define pin_mq2 A1
#define type_mq135 "MQ-135"
#define type_mq2 "MQ-2"
#define ADC_Bit_Resolution 10
#define RatioMQ135CleanAir 3.6
#define RatioMQ2CleanAir 9.83

// Declaramos los sensores
MQUnifiedsensor MQ135(board, Voltage_Resolution, ADC_Bit_Resolution, pin_mq135, type_mq135);
MQUnifiedsensor MQ2(board, Voltage_Resolution, ADC_Bit_Resolution, pin_mq2, type_mq2);

// Creamos un nodo de ROS y dos publishers para enviar el valor de cada sensor
ros::NodeHandle nh;
std_msgs::Float32 mq135_msg;
std_msgs::Float32 mq2_msg;
ros::Publisher mq135_CO2_pub("mq135_CO2", &mq135_msg);
ros::Publisher mq2_GLP_pub("mq2_GLP", &mq2_msg);

void setup()
{
  // Inicializamos la comunicación por puerto serie
  Serial.begin(57600);

  // Inicializamos el sensor MQ-135
  MQ135.init();
  MQ135.setRegressionMethod(1); // PPM = a*ratio^b
  MQ135.setA(110.47); MQ135.setB(-2.862); // Configuramos la ecuación para calcular el valor de la concentración de CO2
  MQ135.setRL(1);

  // Inicializamos el sensor MQ-2
  MQ2.init();
  MQ2.setRegressionMethod(1); // PPM = a*ratio^b
  MQ2.setA(574.25); MQ2.setB(-2.222); // Configuramos la ecuación para calcular el valor de la concentración de GLP
  MQ2.setRL(1);

  /*** Calibración del sensor MQ-135 ***/
  float calcR0_mq135 = 0;
  float calcR0_mq2 = 0;
  for(int i=1; i<=10; i++)
  {
    MQ135.update(); // Actualizamos los datos, el Arduino leerá los valores de voltaje en pin_mq135
    calcR0_mq135 += MQ135.calibrate(RatioMQ135CleanAir);
    MQ2.update(); // Actualizamos los datos, el Arduino leerá los valores de voltaje en pin_mq2
    calcR0_mq2 += MQ2.calibrate(RatioMQ2CleanAir);
  }
  MQ135.setR0(calcR0_mq135/10);
  MQ2.setR0(calcR0_mq2/10);

  if(isinf(calcR0_mq135) || isinf(calcR0_mq2))
  {
    Serial.println(F("R0 es infinito"));
    while(1);
  }
  if(calcR0_mq135 == 0 || calcR0_mq2 == 0)
  {
    Serial.println(F("R0 es cero"));
    while(1);
  }
  /*** Calibración del sensor MQ-135 ***/

  MQ2.serialDebug(true);

  // Inicializamos el nodo de ROS y publicamos los topics "mq135_CO2" y "mq2_GLP"
  nh.initNode();
  nh.advertise(mq135_CO2_pub);
  nh.advertise(mq2_GLP_pub);
}

void loop()
{
  MQ135.update(); // Actualizamos los datos, el Arduino leerá los valores de voltaje en pin_mq135

  float CO2 = MQ135.readSensor(); // El sensor leerá la concentración en PPM usando el modelo (a y b definidos anteriormente)
  Serial.println(CO2 + 400);

  // Asignamos el valor leído al mensaje de ROS y lo publicamos
  mq135_msg.data = CO2 + 400;
  mq135_CO2_pub.publish(&mq135_msg);

  MQ2.update(); // Actualizamos los datos, el Arduino leerá los valores de voltaje en pin_mq2

  float GLP = MQ2.readSensor(); // El sensor leerá la concentración en PPM usando el modelo (a y b definidos anteriormente)
  Serial.println(GLP);

  // Asignamos el valor leído al mensaje de ROS y lo publicamos
  mq2_msg.data = GLP;
  mq2_GLP_pub.publish(&mq2_msg);

  // Procesamos cualquier mensaje entrante y esperamos un breve período antes de volver a leer el sensor
  nh.spinOnce();
  delay(1000);
}

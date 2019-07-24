///////////////
import processing.serial.*;


/////////////////////////////////////////////nova funcao de serial
char ch;
char[] bbuffer = new char[12];  // InvenSense Teapot packet
int serialcont = 0;                 // current packet byte position
int aligned = 0;
int interval = 0;
int ax, ay, az;
float tetax;
boolean flag_x = false;
/////////////////////////////////////////////  
  float dado;
  float x, y;
  float temp, temp_old;
  Serial myPort;        // The serial port
  int xPos = 1;         // horizontal position of the graph
  float inByte = 0;
///////////////////////////////////////////////////////////////////  

int h_graf = 200,l_graf = 700, x_graf= 25, y_graf = 25;
int niveis = 2, tam_eixo = 10;   
char text[];
//eixos y
int x_tex = 10, y_tex = 35;
//float escala = ((float(h_graf)/2)/float(tam_eixo));
Grafico graf1 = new Grafico(25,25, 5, 100);
Grafico graf2 = new Grafico(250,25, 5, 100);

Sinal s1 = new Sinal(graf1);
Sinal s2 = new Sinal(graf2);

Monitor euler_x = new Monitor(0, 50, 200,400,255); // ajustar tamanho dos monitores ** 4/09 - 9:15
Monitor euler_y = new Monitor(50, 400, 200,400,255);
Monitor azimute = new Monitor(400, 400, 200, 400, 255);
String s = "X:";
float angx = 0, angy = 0, angz = 0;
Botao botao_tel = new Botao(10, 10,30, "ini tel");
Botao bot_pwm_up = new Botao(50, 10, 30, "pwm+"); 
Botao bot_pwm_down = new Botao(90, 10, 30, "pwm-");
Botao bot_sinc = new Botao(130,10,30, "sinc");
long tempo_atual = 0, tempo_anterior = 0;
// variaveis do drone
int pwm1, pwm2, pwm3, pwm4; 
float p_gain, i_gain, d_gain, filter_gain;  

void setup(){
  size(1024,700); 
///////////////////////////////////////////////////////////////////////////////////  
      // List all the available serial ports
    // if using Processing 2.1 or later, use Serial.printArray()
    println(Serial.list());

    // I know that the first port in the serial list on my Mac is always my
    // Arduino, so I open Serial.list()[0].
    // Open whatever port is the one you're using.
    myPort = new Serial(this, Serial.list()[0], 9600);


///////////////////////////////////////////////////////////////////////////////////    
  
  fill(0);
  
  //graf1.desenhaGrafico();
  //graf2.desenhaGrafico();
 // myPort.write('T');
}
void draw(){
  botao_tel.showBotao();
  bot_pwm_up.showBotao();
  bot_pwm_down.showBotao();
  bot_sinc.showBotao();
    //graf1.plotSinal(s1);
    //graf2.plotSinal(s2);
    angx = (float)(ax)/10;
    angy = (float)(ay)/10;
    angz = (float)(az)/10;
    euler_x.showEuler(angx, "X:");
    euler_y.showEuler(angy, "Y:");
    azimute.showAzimute(angz, "Z:");
    //print(mouseX,mouseY);  
  }
  
void mousePressed() {
  println("click");
  if (botao_tel.rectOver) {
    if(botao_tel.press_botao == false){
      fill(0,255,0);
      botao_tel.press_botao = true;
      println("botão on");
      myPort.write('t');
    }
    else{
      fill(botao_tel.rectColor);
      println("botao off");
      botao_tel.press_botao = false;
      myPort.write('t');
    }
    
  }
  if(bot_sinc.rectOver){
    println("sincronizando variaveis...");
    sinc_vars();
    }
    
  if (bot_pwm_up.rectOver) {
    println("pwm+");
    myPort.write('p');
   }
  if (bot_pwm_down.rectOver) {
    println("pwm-");
    myPort.write('l');
  }  
}  

void sinc_vars(){
 if(botao_tel.press_botao == false){
     myPort.write("S");
     delay(100);
      if(myPort.read() == '$');
      //verificar se recebe corretamente!
        pwm1 = myPort.read(); 
        pwm4 = myPort.read(); 
        pwm3 = myPort.read(); 
        pwm2 = myPort.read(); 
        
        p_gain = myPort.read(); 
        i_gain = myPort.read(); 
        d_gain = myPort.read(); 
        
        filter_gain = myPort.read(); 
        if(myPort.read() == '#') println("variaveis sincronizadas: " + "\n pwm1:" + pwm1 + "\n pwm3:" + pwm2 + "\n pwm3:" + pwm3 + "\n pwm4:" +pwm4 + "\n P:" + p_gain + "\n I:" + i_gain + "\n D:" + d_gain + "\n filtro:" + filter_gain);
 }
}

void serialEvent(Serial port) {
  
  
    while (port.available() > 0) {
      char ch = (char)port.read();
      if(aligned < 3){  // quadro ainda não está alinhado
        if(serialcont == 0){ 
         if(ch == '$'){
           aligned ++;
           bbuffer[serialcont] = ch;
           println("$ encontrado");
         }else aligned = 0;
        }else if(serialcont == 1){ 
          if(ch == 'X'){
            aligned ++; 
            bbuffer[serialcont] = ch;
            println("X encontrado");
          }else aligned = 0;
          }else if(serialcont == 11){ 
          if(ch == '\n'){
            aligned ++; 
            bbuffer[serialcont] = ch;
            println("\n encontrado");
          }else aligned = 0;
        }
        println(hex(ch) + " " + aligned + " " + serialcont);
        serialcont++;
        if (serialcont == 11) serialcont = 0; 
      }else{
        if (serialcont > 0 || ch == '$') {
          //println(hex(ch) + "        " + serialcont);
          if(ch == 'Y' && serialcont == 3) bbuffer[serialcont++] = 0;
          if(ch == 'Z' && serialcont == 6) bbuffer[serialcont++] = 0;
           if(ch == '#' && serialcont == 9) bbuffer[serialcont++] = 0;
          bbuffer[serialcont++] = ch;
          if (serialcont == 12) {
            
            for(int i = 0; i < 11; i++) print(hex(bbuffer[i]) + "  |  ");
            
            ax =  ((int)(bbuffer[3] << 8) | bbuffer[2]);
            if((bbuffer[3]  & 128) != 0) ax = ax ^ 0xFFFF0000;
            
            ay = ((int)(bbuffer[6] << 8) | bbuffer[5]);
            if((bbuffer[6]  & 128) != 0) ay = ay ^ 0xFFFF0000; 
            
            az = ((int)(bbuffer[9] << 8) | bbuffer[8]);
            if((bbuffer[9]  & 128) != 0) ay = ay ^ 0xFFFF0000;
            
            print("X:" + ((float)(ax)/10) + "   Xhex: " + hex(ax));
            print("   Y:" + ((float)(ay)/10));
            println("   Z:" + ((float)(az)/10));
            serialcont = 0; // restart packet byte position
          }
        }  

    }
   }
}

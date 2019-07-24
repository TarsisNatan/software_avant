class Botao{ 
 int rectX, rectY;      // Position of square button
int rectSize;     // Diameter of rect
color rectColor, baseColor;
color rectHighlight;
color currentColor;
String name;
boolean rectOver, press_botao;

public Botao(int x, int y, int tam, String n) {
  rectColor = color(0);
  rectHighlight = color(51);
  baseColor = color(102);
  currentColor = baseColor;
  rectX = x;
  rectY = y;
  rectSize = tam;
  rectOver = false;
  press_botao = false;
  name = n;
}

void showBotao() {
  update(mouseX, mouseY);
  
  if (rectOver) {
    fill(rectHighlight);
  } else {
    fill(rectColor);
  }
  stroke(255);
  rect(rectX, rectY, rectSize, rectSize);
  fill(255);
  text(name,rectX, rectY + rectSize);
}

void update(int x, int y) {
  if ( overRect(rectX, rectY, rectSize, rectSize) ) {
    rectOver = true;
  } else {
    rectOver = false;
  }
}

void mousePressed() {
  println("click");
  if (rectOver) {
    if(press_botao == false){
    fill(0,255,0);
    press_botao = true;
    println("botão on");
    }
    else{
      fill(rectColor);
      println("botao off");
      press_botao = false;
    }
    
  }
}

boolean overRect(int x, int y, int width, int height)  {
  if (mouseX >= x && mouseX <= x+width && 
      mouseY >= y && mouseY <= y+height) {
    return true;
  } else {
    return false;
  }
}
  
  
}

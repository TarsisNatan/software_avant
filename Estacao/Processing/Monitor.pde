class Monitor{
 // atributos
 int xo, yo, xf, yf, alt, comp; 
 color cor_fundo;
 //////metodos/////
 //construtores 
 public Monitor(int x, int y, int a, int c, color cor){
   xo = x;
   yo = y; 
   alt = a; 
   comp = c; 
   cor_fundo = cor;
   xf = xo + comp; 
   yf = yo + alt;
 }
 //interface
 public void showEuler(float ang, String s){
   int xol, yol;
   float xfl1, xfl2, yfl, yfl2;
   float raio;
   showMonitor();
   raio = (alt/2); 
   xol = comp/2 + xo; 
   yol = alt/2 + yo;
   xfl1 = (cos(radians(ang))*raio + (xo + comp/2));
   yfl = (-sin(radians(ang))*raio + (yo + alt/2));
   xfl2 = (-cos(radians(ang))*raio + (xo + comp/2));
   yfl2 = (sin(radians(ang))*raio + (yo + alt/2));
   fill(0);
   line(xol, yol, xfl1, yfl );
   line(xol, yol, xfl2, yfl2 );
   fill(255,0,0);
   text(s, xo, yf);
   printVal_f(xo + 10, yf, ang);
   }
   
   public void showAzimute(float ang, String s){
   int xol, yol;
   float xfl1, xfl2, yfl1, yfl2, xfl3, xfl4, yfl3, yfl4;
   float raio;
   showMonitor();
   raio = (alt/2); 
   xol = comp/2 + xo; 
   yol = alt/2 + yo;
   
   xfl1 = (cos(radians(ang))*raio + (xo + comp/2));
   yfl1 = (-sin(radians(ang))*raio + (yo + alt/2));
   xfl2 = (-cos(radians(ang))*raio + (xo + comp/2));
   yfl2 = (sin(radians(ang))*raio + (yo + alt/2));
   
   yfl3 = (cos(radians(ang))*raio + (yo + alt/2));
   xfl3 = (sin(radians(ang))*raio + (xo + comp/2));
  
   yfl4 = (-cos(radians(ang))*raio + (yo + alt/2));
   xfl4 = (-sin(radians(ang))*raio + (xo + comp/2));
   
   //line(xol, yol, xfl1, yfl1 );
   //line(xol, yol, xfl2, yfl2 );
   line(xol, yol, xfl3, yfl3 );
   //line(xol, yol, xfl4, yfl4 );
   fill(255,0,0);
   text(s, xo, yf);
   printVal_f(xo + 10, yf, ang);
   }
 //privados
 private void showMonitor(){
  fill(cor_fundo); 
  stroke(3);
  rect(xo, yo, comp, alt); 
 }
 private void printVal_f(int x, int y, float val_f){ 
  text(val_f, x, y); 
 }
 
}

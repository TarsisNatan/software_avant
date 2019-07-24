class Grafico{ 
  private int h_g,l_g, x_g, y_g; // dimensões do retângulo do gráfico em pixels 
  private int n, t_eixo; // nivel: zoom, tam_eixo: magnitude do eixo y 
  private int x_tex, y_tex;  // coordenadas do eixo Y
  private float escala;;
  //construtor
  public Grafico(int yi, int xi, int n, int t){
      this.h_g = 200;
      this.l_g = 700; 
      this.x_g= xi; 
      this.y_g = yi;  
      this.n = n; 
      this.t_eixo = t;  
      this.x_tex = xi - 15; 
      this.y_tex = yi;  
      
 }
 private void plotSinal(Sinal s){
   escala = ((float(h_g)/2)/float(t_eixo));
   s.desenhaSinal(escala, t_eixo);
   if (s.xpos >= s.xmax) limpaTela(); 
       
   
 }
 public void desenhaGrafico(){ 
   int i = t_eixo;
    fill(255, 255,255);
    rect(x_g, y_g, l_g, h_g);  
    //eixos dos graficos
    fill(255, 255,255);
    while(i >= -t_eixo){
      text(i,this.x_tex,this.y_tex);
      i = i - (t_eixo/n);
      y_tex = y_tex + h_g/(2*n);
    }
    //eixo x
    fill(0);
    line(this.x_g, (this.y_g + this.h_g/2), (this.l_g + this.x_g), (this.y_g + this.h_g/2));
   // plotSinal(s1);
 
 }
 public void limpaTela(){ 
   fill(255,255,255);
   rect(this.x_g, this.y_g, this.l_g, this.h_g); 
 }
}
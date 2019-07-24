class Sinal{
  private int xpos, yi, yf, xi; // o sinal é composto por um conjunto de retas, que iniciam no eixo x_graf(pontos:xi, yi) e termina na magnitude do sinal medido ao decorrer do eixo y_graf (pontos :xf,yf)
  private int xmax, ymax; // tamanho do shape para aparecer na tela
  private float d; //dados do sinal
  //construtor
  public Sinal(int xi, int y, int xm){
    this.xi = xi;
    this.yi = y;  
    this.xmax = xm;
    this.xpos = xi;
  }
  public Sinal(Grafico g){
    // sinal inicia na origem do gráfico
    this.xi = g.x_g;
    this.yi = g.y_g + g.h_g/2;
    // shape máximo do sinal é a posição da largura do gráfico e altura
    this.xmax = g.l_g + g.x_g;
    this.ymax = g.h_g + g.y_g;
    xpos = xi;
    
  }
  public void setData(float valor){ 
    this.yf = int(valor);
  }
  public float getData(){ 
    return this.yf; 
  }
  
  public void desenhaSinal(float escala, int t){ 
    if (this.xpos > this.xmax) {
     // desenhaEixos(tam_eixo, x_tex, y_tex, niveis, h_graf, y_graf, x_graf, l_graf);
      this.xpos = this.xi;
    } else {
      // increment the horizontal position:
      this.xpos++;
    }
    stroke(127, 34, 255);
    setData(dado); // variável "dado" do sensor global = yf
    // desenha conjunto de retas para representar o sinal no tempo
    line(this.xpos, this.yi, this.xpos, (this.yi - this.yf*escala));
  }
}
  
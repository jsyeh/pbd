//My implementation of Position Based Dynamics
//https://matthias-research.github.io/pages/publications/posBasedDyn.pdf
//現在想試試 stretching，讓兩線段，可設定「特定」的長度
class Particle {
  PVector x; //position
  PVector p; //estimated position in simulation (predicted position)
  PVector v; //velocity
  boolean locked;
  float w;
  Particle( float _x, float _y, float _z, float _vx, float _vy, float _vz ) {
    x = new PVector(_x, _y, _z);
    p = new PVector(_x, _y, _z);
    v = new PVector(_vx, _vy, _vz);
    locked = false;
    w = 1;
  }
  Particle( float _x, float _y, float _z ) {
    this( _x, _y, _z, 0.0, 0.0, 0.0);
  }
  Particle( float _x, float _y, float _vx, float _vy ) {
    this( _x, _y, random(-0.1, +0.1), _vx, _vy, 0.0);
  } //z 值 random() 擾動,可讓布料跳脫原本2D世界, 讓布的變化變立體!!!
  Particle( float _x, float _y ) {
    this( _x, _y, 0.0, 0.0 );
  }
  void draw() {
    float r = 3; //要畫圓球時的半徑
    noStroke();
    if(locked) fill(#FF0000);
    else fill(#FFFF00);
    pushMatrix();
      translate(x.x, x.y, x.z);
      sphere(r);
      
      translate(100, 0, 0);
      fill(#FF0000);
      if(locked) sphere(5);
      else sphere(3);
    popMatrix();
  }
}
class Stick {
  Particle [] p = new Particle[2];
  float r = 3; //要畫圓柱時的半徑
  Stick( Particle _p1, Particle _p2 ) {
    p[0] = _p1;
    p[1] = _p2;
  }
  void draw() {
    float r = 3; //要畫圓柱時的半徑
    PVector p0 = p[0].x, p1 = p[1].x;
    PVector V = PVector.sub(p1,p0), X = new PVector(1,0,0);
    PVector Z = V.cross(X).normalize(), V2 = Z.cross(V).normalize();
    noStroke(); //stroke(255, 255, 0);
    fill(255, 255, 0);
    beginShape(QUAD_STRIP); // 要畫出圓柱
    for(float a=0; a<PI*2; a+=0.1){
      PVector d = PVector.add( PVector.mult(Z,cos(a)*r), PVector.mult(V2,sin(a)*r) ); 
      vertex(p0.x+d.x, p0.y+d.y, p0.z+d.z);
      vertex(p1.x+d.x, p1.y+d.y, p1.z+d.z);
    }
    endShape(CLOSE);
    
    stroke(255, 0, 0); //在右邊比較的版本，只用 line()
    line(p0.x+100, p0.y, p0.z, p1.x+100, p1.y, p1.z);
  }
}
class Stretch {
  Particle p1, p2;
  float len;
  Stretch( Stick s ) {
    this( s.p[0], s.p[1] );
  }
  Stretch( Particle _p1, Particle _p2 ) {
    p1 = _p1;
    p2 = _p2;
    len = PVector.dist(p1.x, p2.x);
  }
  float C() {
    return PVector.dist(p1.p, p2.p) - len;
  }
  void projectConstraint() {
    float stiffness = 1; //照著 section 3.3 最後整合 solverIterations 及 stiffness
    int ns = 20; // number of solverIterations
    float k2 = 1 - pow(1-stiffness, 1.0/ns); //照著 section 3.3 最後的公式, 算出k' 
    float C = C();
    PVector n = PVector.sub(p1.p, p2.p).normalize();
    PVector dp1 = PVector.mult(n, -p1.w/(p1.w+p2.w)*C*k2);
    PVector dp2 = PVector.mult(n, +p2.w/(p1.w+p2.w)*C*k2);
    if(!p1.locked) p1.p.add(dp1);
    if(!p2.locked) p2.p.add(dp2);
  }
}

ArrayList<Particle> particles = new ArrayList<Particle>();
ArrayList<Stick> sticks = new ArrayList<Stick>();
ArrayList<Stretch> stretches = new ArrayList<Stretch>();

void setup() {
  size(500, 500, P3D);
}

void draw() {
  lights();
  background(#FFFFF2);
  textSize(20);
  fill(0);
  text("Left dragged: draw curve rope", 50, 40);
  text("Right dragged: move top vertex", 50, 70);
  //translate(width/2, height/2); //mouse畫線的座標已好，不用再移
  //rotateY(radians(30)); //不要旋轉，以便2D的視角觀察
  for( Stick s : sticks ) {
    s.draw();
  }
  for( Particle p : particles ) {
    p.draw();
  }
  //if(keyPressed || ! mousePressed) Simulation();
  Simulation(); //可一直持續模擬
}

void keyPressed() { //

}

void mousePressed() {
  if(mouseButton==LEFT) { //左鍵，記下開始點，準備拖曳
    Particle now = new Particle(mouseX, mouseY);
    now.locked = true;
    now.w = 0; //3.6 Attachment 要記得將w設成0才會固定
    //To make sure other constraints containing this vertex
    //do not move it, its inverse mass wi is set to zero.
    particles.add(now);    
  }
}

void mouseDragged() { //拖曳上方的頂點，並即時模擬
  if(mouseButton==LEFT) {
    Particle prev = particles.get(particles.size()-1);
    PVector p = prev.x;
    float d = 10; //這是 particles 的距離
    if(dist(p.x, p.y, p.z, mouseX, mouseY, 0) > d){
      PVector v = new PVector(mouseX-p.x, mouseY-p.y).normalize();
      Particle now = new Particle(p.x+d*v.x, p.y+d*v.y);
      particles.add(now);
      Stick s = new Stick(prev,now);
      sticks.add(s);
      stretches.add(new Stretch(s));
    }
  }else if(mouseButton==RIGHT&&particles.size()>0) {
    int lastI = particles.size()-1;
    Particle last = particles.get(lastI);
    last.x.add(new PVector(mouseX-pmouseX,mouseY-pmouseY));
    last.p.add(new PVector(mouseX-pmouseX,mouseY-pmouseY));
    last.locked = true;
    last.w = 0;
  }
}

void Simulation() {
  int ns = 20; //模擬的 number of simulation 有重覆要小心
  PVector gravity = new PVector(0, 0.98, 0);
  for( Particle p : particles ) {
    if(p.locked) continue; 
    p.v.add( gravity ); //先不處理gravity
    p.v.mult(0.999);  //暫不處理位置、速度、加速度的更新
    p.p = PVector.add(p.x, p.v);
  }
  for(int k=0; k<ns; k++) { //模擬的 number of simulation
    projectConstraints();
  }
  for( Particle p: particles ) {
    p.v = PVector.sub(p.p, p.x); //先不更新速度
    p.x.x = p.p.x;
    p.x.y = p.p.y;
    p.x.z = p.p.z;
  }
}

void projectConstraints() {
  for( Stretch s : stretches ) {
    s.projectConstraint();
  }
}

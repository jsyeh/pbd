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
    noStroke();
    fill(#FF0000);
    pushMatrix();
      translate(x.x, x.y, x.z);
      sphere(7);
    popMatrix();
  }
}
class Stick {
  Particle [] p = new Particle[2];
  Stick( Particle _p1, Particle _p2 ) {
    p[0] = _p1;
    p[1] = _p2;
  }
  void draw() {
    PVector p0 = p[0].x, p1 = p[1].x;
    stroke(255, 0, 0);
    line(p0.x, p0.y, p0.z, p1.x, p1.y, p1.z);
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
    float stiffness = 0.99; //照著 section 3.3 最後整合 solverIterations 及 stiffness
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
  generateTwoTriangle();
}

void draw() {
  lights();
  background(#FFFFF2);
  translate(width/2, height/2);
  rotateY(radians(30));
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

void mouseDragged() { //拖曳上方的頂點，並即時模擬
  particles.get(2).x.add(new PVector(mouseX-pmouseX,mouseY-pmouseY));
  particles.get(2).p.add(new PVector(mouseX-pmouseX,mouseY-pmouseY));
}

void generateTwoTriangle() {
  Particle p3 = new Particle(100, -150, 0); //上方
  Particle p1 = new Particle(0, 0, -100), p2 = new Particle(0, 0, 100); //中間
  Particle p4 = new Particle(100, 150, 0); //下方
  p3.locked = true; //固定1個點，以便觀察
  p3.w = 0; // 也要設成0 才會在 projectConstraint() 時真固定
  particles.add(p1);
  particles.add(p2);
  particles.add(p3);
  particles.add(p4);
  sticks.add(new Stick(p1,p2));
  sticks.add(new Stick(p1,p3));
  sticks.add(new Stick(p2,p3));
  sticks.add(new Stick(p1,p4));
  sticks.add(new Stick(p2,p4));
  for( Stick s : sticks ) {
    stretches.add(new Stretch(s));
  }
}

void Simulation() {
  int ns = 20; //模擬的 number of simulation 有重覆要小心
  PVector gravity = new PVector(0, 0.98, 0);
  for( Particle p : particles ) {
    if(p.locked) continue; 
    p.v.add( gravity ); //處理gravity
    p.v.mult(0.999);  //處理位置、速度、加速度的更新
    p.p = PVector.add(p.x, p.v);
  }
  for(int k=0; k<ns; k++) { //模擬的 number of simulation
    projectConstraints();
  }
  for( Particle p: particles ) {
    p.v = PVector.sub(p.p, p.x); //更新速度
    p.x.x = p.p.x; //更新位置
    p.x.y = p.p.y;
    p.x.z = p.p.z;
  }
}

void projectConstraints() {
  for( Stretch s : stretches ) {
    s.projectConstraint();
  }
}

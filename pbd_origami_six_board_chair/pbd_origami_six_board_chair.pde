//My implementation of Position Based Dynamics
//https://matthias-research.github.io/pages/publications/posBasedDyn.pdf
//現在想試試 stretch 及 bending，讓6個四邊形，可設定「特定」的角度&形狀
//而每個四邊形，可從2個三角形構成，我要先做出2個四邊形
//有些bend的角度可改，有些bend的角度不能改
//Q：但要怎麼解決「兩方向」角度的問題？
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

class Rectangle {
  Particle [] p = new Particle[4];
  Rectangle( Particle _p0, Particle _p1, Particle _p2, Particle _p3 ) {
    p[0] = _p0;
    p[1] = _p1;
    p[2] = _p2;
    p[3] = _p3;
    for(int i=0; i<4; i++) {
      stretches.add(new Stretch(p[i], p[(i+1)%4]));
    }
    stretches.add(new Stretch(p[0], p[2]));
    stretches.add(new Stretch(p[1], p[3]));
  }
  void draw() {
    PVector p0 = p[0].x, p1 = p[1].x, p2 = p[2].x, p3 = p[3].x;
    fill(255, 0, 0, 128); stroke(0);
    beginShape();
      vertex(p0.x, p0.y, p0.z);
      vertex(p1.x, p1.y, p1.z);
      vertex(p2.x, p2.y, p2.z);
      vertex(p3.x, p3.y, p3.z);
    endShape();
  }
}

class Triangle {
  Particle [] p = new Particle[3];
  Triangle( Particle _p0, Particle _p1, Particle _p2 ) {
    p[0] = _p0;
    p[1] = _p1;
    p[2] = _p2;
    for(int i=0; i<3; i++) {
      stretches.add(new Stretch(p[i], p[(i+1)%3]));
    }
  }
  void draw() {
    PVector p0 = p[0].x, p1 = p[1].x, p2 = p[2].x;
    fill(255, 0, 0, 128); stroke(0);
    beginShape();
      vertex(p0.x, p0.y, p0.z);
      vertex(p1.x, p1.y, p1.z);
      vertex(p2.x, p2.y, p2.z);
    endShape(CLOSE);    
  }
}

class Bend {
  Particle p1, p2, p3, p4;
  PVector n1, n2;
  float theta0, theta;
  Bend( Particle _p1, Particle _p2, Particle _p3, Particle _p4) {
    p1 = _p1; //中間
    p2 = _p2; //中間
    p3 = _p3;
    p4 = _p4;
    n1 = PVector.sub(p2.x,p1.x).cross(PVector.sub(p3.x,p1.x)).normalize();
    n2 = PVector.sub(p2.x,p1.x).cross(PVector.sub(p4.x,p1.x)).normalize();
    theta0 = PVector.angleBetween(n1, n2);
    println(theta0);
  }
  float C() {
    n1 = PVector.sub(p2.p,p1.p).cross(PVector.sub(p3.p,p1.p)).normalize();
    n2 = PVector.sub(p2.p,p1.p).cross(PVector.sub(p4.p,p1.p)).normalize();
    println(PVector.angleBetween(n1, n2));
    return PVector.angleBetween(n1, n2) - theta0;
  }
  void projectConstraint() {
    PVector pp2 = PVector.sub(p2.p,p1.p), pp3 = PVector.sub(p3.p,p1.p), pp4 = PVector.sub(p4.p,p1.p);
    n1 = pp2.cross(pp3).normalize();
    n2 = pp2.cross(pp4).normalize();
    float d = PVector.dot(n1,n2);
    float p2xp3mag = pp2.cross(pp3).mag(), p2xp4mag = pp2.cross(pp4).mag();
    PVector q3 = PVector.add(pp2.cross(n2), PVector.mult(n1.cross(pp2),d)).div(p2xp3mag);
    PVector q4 = PVector.add(pp2.cross(n1), PVector.mult(n2.cross(pp2),d)).div(p2xp4mag);
    PVector q2 = PVector.add( PVector.add(pp3.cross(n2), PVector.mult(n1.cross(pp3),d)).div(p2xp3mag) , PVector.add(pp4.cross(n1), PVector.mult(n2.cross(pp4),d)).div(p2xp4mag) ).mult(-1);
    PVector q1 = PVector.add(PVector.add(q2,q3),q4).mult(-1);
    float mother = q1.magSq() + q2.magSq() + q3.magSq() + q4.magSq();
    if(mother<0.0000000001) return;
    float param = - sqrt(1-d*d) * C() / mother;
    if(!p1.locked) p1.p.add(PVector.mult(q1,param));
    if(!p2.locked) p2.p.add(PVector.mult(q2,param));
    if(!p3.locked) p3.p.add(PVector.mult(q3,param));
    if(!p4.locked) p4.p.add(PVector.mult(q4,param));
  }
}

ArrayList<Particle> particles = new ArrayList<Particle>();
ArrayList<Stick> sticks = new ArrayList<Stick>();
ArrayList<Stretch> stretches = new ArrayList<Stretch>();
ArrayList<Triangle> triangles = new ArrayList<Triangle>();
ArrayList<Bend> bends = new ArrayList<Bend>();
ArrayList<Rectangle> rectangles = new ArrayList<Rectangle>();

void setup() {
  size(500, 500, P3D);
  generateSixBoardChair();
}

void draw() {
  lights();
  background(#FFFFF2);
  textSize(20);
  fill(0);
  text("+: increase bending angle", 50, 40);
  text("-: decrease bending angle", 50, 70);

  translate(width/2, height/2);
  rotateY(radians(30));
  for( Triangle t : triangles ) {
    t.draw();
  }
  for( Rectangle r : rectangles ) {
    r.draw();
  }
  for( Stick s : sticks ) {
    s.draw();
  }
  for( Particle p : particles ) {
    p.draw();
  }
  //if(keyPressed || ! mousePressed) Simulation(); //按鍵、放開滑鼠：模擬
  //else if(mousePressed && mouseButton==LEFT) Simulation(); //右鍵移動：模擬
  //意思是，左鍵（暫改變角度）時，不模擬
  Simulation();
}

void keyPressed() { //按按鍵，可調整 theta0 預設角度
  if(key=='-' && bends.get(0).theta0 > 0.1) { //按'0'，會減少角度
    bends.get(0).theta0 -= 0.1; //修改bending 的預設角度
  }else if(bends.get(0).theta0 < PI - 0.1) { //其他鍵，都會增加角度
    bends.get(0).theta0 += 0.1; //修改bending 的預設角度
  }
}

void mouseDragged() { //拖曳上方的頂點，並即時模擬
  if(mouseButton==RIGHT) { // 右鍵「左右拖曳」移動（上方）頂點位置
    particles.get(0).x.add(new PVector(mouseX-pmouseX,mouseY-pmouseY));
    particles.get(0).p.add(new PVector(mouseX-pmouseX,mouseY-pmouseY));
    particles.get(3).x.add(new PVector(mouseX-pmouseX,mouseY-pmouseY));
    particles.get(3).p.add(new PVector(mouseX-pmouseX,mouseY-pmouseY));
  } else if(mouseButton==LEFT) { //左鍵「左右拖曳」可暫改變（下方）座標
    PVector p0 = particles.get(0).x, p1 = particles.get(2).x;
    float len = dist(p0.x, p0.y, p1.x, p1.y);
    float diff = mouseY - pmouseY;
    PVector v = PVector.sub(p1,p0).mult((len+diff)/len); //增加長度
    particles.get(2).x = PVector.add(p0, v);
    particles.get(2).p = PVector.add(p0, v);
    particles.get(2).locked = true; //先固定、鎖住下方的點，讓它不要模擬
    particles.get(2).w = 0; //先固定、鎖住下方的點，讓它不要模擬
    particles.get(9).x = PVector.add(particles.get(7).x, v);
    particles.get(9).p = PVector.add(particles.get(7).x, v);
    particles.get(9).locked = true; //先固定、鎖住下方的點，讓它不要模擬
    particles.get(9).w = 0; //先固定、鎖住下方的點，讓它不要模擬
  }
}

void mouseReleased() { //放開時，再解鎖下方的點，讓它可以模擬
  particles.get(2).locked = false;
  particles.get(2).w = 1;
  particles.get(9).locked = false;
  particles.get(9).w = 1;
}

void generateSixBoardChair() { //要處理「不能轉」及「能轉」的Hindge
  Particle p1 = new Particle(-200, -100); //左上，z座標都會放亂數
  Particle p2 = new Particle(-200, 0); //左
  Particle p3 = new Particle(-150, +150); //左下
  Particle p4 = new Particle(0, -100); //上
  Particle p5 = new Particle(-50, 0); //中左
  Particle p6 = new Particle(0, +100); //下
  Particle p7 = new Particle(+50, 0); //中左
  Particle p8 = new Particle(200, -100); //右上
  Particle p9 = new Particle(200, 0); //右
  Particle p10 = new Particle(150, +150); //右下
  p4.locked = true; //固定以便觀察
  p4.w = 0; // 也要設成0 才會在 projectConstraint() 時真固定
  p5.locked = true; //固定以便觀察
  p5.w = 0; // 也要設成0 才會在 projectConstraint() 時真固定
  p7.locked = true; //固定以便觀察
  p7.w = 0; // 也要設成0 才會在 projectConstraint() 時真固定
  particles.add(p1);
  particles.add(p2);
  particles.add(p3);
  particles.add(p4);
  particles.add(p5);
  particles.add(p6);
  particles.add(p7);
  particles.add(p8);
  particles.add(p9);
  particles.add(p10);
  rectangles.add(new Rectangle(p1,p2,p5,p4));
  rectangles.add(new Rectangle(p2,p3,p6,p5));
  rectangles.add(new Rectangle(p4,p7,p9,p8));
  rectangles.add(new Rectangle(p7,p6,p10,p9));
  triangles.add(new Triangle(p4,p5,p7));
  triangles.add(new Triangle(p5,p6,p7));
  bends.add(new Bend(p5,p7,p4,p6)); //可彎曲的面
  bends.get(0).theta0 -= 0.2;
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
  for( Bend b : bends ) {
    b.projectConstraint();
  }
}

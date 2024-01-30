//My implementation of Position Based Dynamics
//https://matthias-research.github.io/pages/publications/posBasedDyn.pdf
//現在想試試 bending，讓兩塊三角形，可以設定「特定」的角度
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
}
class Triangle {
  Particle [] p = new Particle[3];
  Triangle( Particle _p0, Particle _p1, Particle _p2 ) {
    p[0] = _p0;
    p[1] = _p1;
    p[2] = _p2;
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
    p1.p.add(PVector.mult(q1,param));
    p2.p.add(PVector.mult(q2,param));
    p3.p.add(PVector.mult(q3,param));
    p4.p.add(PVector.mult(q4,param));
  }
}
ArrayList<Particle> particles = new ArrayList<Particle>();
ArrayList<Triangle> triangles = new ArrayList<Triangle>();
ArrayList<Bend> bends = new ArrayList<Bend>();
void setup() {
  size(500, 500, P3D);
  generateTwoTriangle();
}
void draw() {
  background(#FFFFF2);
  translate(width/2, height/2);
  rotateY(radians(30));
  for( Triangle t : triangles ) {
    PVector p0 = t.p[0].x, p1 = t.p[1].x, p2 = t.p[2].x;
    fill(255, 0, 0, 128); stroke(0);
    beginShape();
      vertex(p0.x, p0.y, p0.z);
      vertex(p1.x, p1.y, p1.z);
      vertex(p2.x, p2.y, p2.z);
    endShape(CLOSE);
  }
  for( Particle p : particles ) {
    PVector pt = p.x;
    noStroke();
    fill(#FF0000);
    pushMatrix();
      translate(pt.x, pt.y, pt.z);
      sphere(7);
    popMatrix();
  }
  //if(keyPressed || ! mousePressed) Simulation();
  //if(keyPressed) Simulation();
}
void keyPressed() {
  Simulation();
}
void mouseDragged() {
  particles.get(3).x.x += mouseX-pmouseX;
  particles.get(3).p.x += mouseX-pmouseX;
  //particles.get(3).x.x = 150*cos(radians(mouseX));//+= mouseX-pmouseX;
  //particles.get(3).x.y = 150*sin(radians(mouseX));//+= mouseX-pmouseX;
  //particles.get(3).p.x = 150*cos(radians(mouseX));//+= mouseX-pmouseX;
  //particles.get(3).p.y = 150*sin(radians(mouseX));//+= mouseX-pmouseX;
}
void generateTwoTriangle() {
  Particle p3 = new Particle(100, -150, 0);
  Particle p1 = new Particle(0, 0, -100), p2 = new Particle(0, 0, 100); //中間
  Particle p4 = new Particle(100, 150, 0);
  particles.add(p1);
  particles.add(p2);
  particles.add(p3);
  particles.add(p4);
  triangles.add(new Triangle(p1,p2,p3));
  triangles.add(new Triangle(p1,p2,p4)); 
  bends.add(new Bend(p1,p2,p3,p4));
}
void Simulation() {
  int ns = 1;
  for( Particle p : particles ) {
    if(p.locked) continue; 
    //p.v.add( gravity );
    //p.v.mult(0.9);
    //p.p = PVector.add(p.x, p.v);
  }
  for(int k=0; k<ns; k++) {
    projectConstraints();
  }
  for( Particle p: particles ) {
    p.v = PVector.sub(p.p, p.x);
    p.x.x = p.p.x;
    p.x.y = p.p.y;
    p.x.z = p.p.z;
  }
}

void projectConstraints() {
  for( Bend b : bends ) {
    b.projectConstraint();
  }
}

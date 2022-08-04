//My implementation of Position Based Dynamics
//https://matthias-research.github.io/pages/publications/posBasedDyn.pdf
class Particle{
  PVector x; //position
  PVector p; //estimated position in simulation
  PVector v; //velocity
  boolean locked;
  float w;
  Particle( float _x, float _y, float _z, float _vx, float _vy, float _vz ){
    x = new PVector(_x, _y, _z);
    p = new PVector(_x, _y, _z);
    v = new PVector(_vx, _vy, _vz);
    locked = false;
    w = 1;
  }
  Particle( float _x, float _y, float _vx, float _vy ){
    this( _x, _y, random(-0.1,+0.1), _vx, _vy, 0.0);
  } //z 值 random() 擾動,可讓布料跳脫原本2D世界, 讓布的變化變立體!!!
  Particle( float _x, float _y ){
    this( _x, _y, 0.0, 0.0 );
  }
}
class Stick{
  float len;
  Particle p1, p2;
  Stick( Particle _p1, Particle _p2 ){
    p1 = _p1;
    p2 = _p2;
    len = PVector.dist(p1.x, p2.x);
  }
}
class Triangle{
  Particle [] p = new Particle[3];
  Triangle( Particle _p0, Particle _p1, Particle _p2 ){
    p[0] = _p0;
    p[1] = _p1;
    p[2] = _p2;
  }
}
ArrayList<Stick> sticks;
ArrayList<Particle> particles;
ArrayList<Triangle> triangles;
PVector sphere; //用來研究Collision Constraint 的大球
void setup(){
  size(500, 500, P3D); //為了打光lights(), 所以改成 P3D
  generateCloth(400, 400, 20, 20); //這裡可調解析度,方便debug
  sphere = new PVector(0, 0, -100); //有一個大球, 用來研究 Collision Constraint
}
void draw(){
  lights(); //想加上打光,不過三角面用半透明的話,有點怪怪的
  background(#FFFFF2);
  translate(width/2, height/2);
  pushMatrix();
    translate(sphere.x, sphere.y, sphere.z);
    noStroke();
    fill(255);
    sphere(100);
  popMatrix();
  if(Q_c!=null){ //有collision發生時, 用小圓球把表面接觸點標示出來
    for( PVector q_c : Q_c ){
      pushMatrix();
        translate(q_c.x, q_c.y, q_c.z);
        fill(255,0,0);
        sphere(5);
      popMatrix();
    }
  }
  for( Triangle t : triangles ){
    PVector p0 = t.p[0].x, p1 = t.p[1].x, p2 = t.p[2].x;
    fill(255, 0, 0, 128); noStroke();
    //triangle( p0.x, p0.y, p1.x, p1.y, p2.x, p2.y );
    beginShape(); //因為想畫出3D的效果, 所以把 triangle()改成 vertex()
      vertex(p0.x, p0.y, p0.z);
      vertex(p1.x, p1.y, p1.z);
      vertex(p2.x, p2.y, p2.z);
    endShape();
  }
  for( Stick s : sticks ){
    PVector p1 = s.p1.x, p2=s.p2.x;
    stroke(255,0,0);
    line( p1.x, p1.y, p1.z, p2.x, p2.y, p2.z); //因為想變3D效果,所以加上z座標
  }
  for( Particle p : particles ){
    PVector pt = p.x;
    stroke(0);
    if(p.locked){
      fill(#FF0000);
      ellipse( pt.x, pt.y, 7, 7); //固定的點畫大一點(locked)
    }
  }
  if(mousePressed) Simulation();
}
void Simulation(){
  float stiffness = 0.98; //照著 section 3.3 最後整合 solverIterations 及 stiffness
  int ns = 10; // number of solverIterations
  float k2 = 1 - pow(1-stiffness, 1.0/ns); //照著 section 3.3 最後的公式, 算出k' 
  PVector gravity = new PVector(0, 0.98, 0); //像地球 vs.月球: 重力不同, 布飄動的結果不同
  for( Particle p : particles ){ //update external force
    if(p.locked) continue; //skip locked particle
    p.v.add( gravity ); //(5) vi += dt*wi*fext(xi)
    p.v.mult(0.9); //先變小//(6) dampVelocities()
    p.p = PVector.add(p.x, p.v); //(7) pi = xi + dt*vi
  }
  generateCollisionConstraints(); //(8) generateCollisionConstraints()
  for(int k=0; k<ns; k++){ //(9) solverIterations 越多次,越剛直, 所以用k2來修正回來
    projectConstraints(k2); //(10) projectConstraints() in Gauss-Seidel fashion
  }
  for( Particle p : particles ){//(12) forall vertices i
    p.v = PVector.sub(p.p, p.x); //(13) vi = (pi-xi)/dt
    p.x.x = p.p.x; //(14) xi = pi
    p.x.y = p.p.y;
    p.x.z = p.p.z;
  }
  //(16) velocityUpdate()
}
ArrayList<PVector> Q_c=null;
void generateCollisionConstraints(){
  if(Q_c==null) Q_c = new ArrayList<PVector>();
  else Q_c.removeAll(Q_c);
  for( Particle p : particles ){
    if( PVector.dist(p.p, sphere)<100 ){ //estimated position 在圓球裡
      PVector q_c = calcContactPoint(sphere, p.x, p.p); //2種collision都在函式中解決
      Q_c.add(q_c);
    }
  }
  println(Q_c.size());
}
PVector calcContactPoint(PVector sphere, PVector x0, PVector p){ //已知: estimated position 在圓球裡
  //q_c contact point, n_c normal at contact point (continuous collision) 一裡一外
  //q_s surface point, n_s normal at surface point (static collision) 兩點都在裡面
  if( PVector.dist(x0, sphere)<100 ){ //position 也在圓球裡, 用 static collision
    print("+"); //static collision
    PVector q_c = PVector.add( sphere, PVector.sub(p, sphere).normalize().mult(100) );//找p最近的圓球表面
    return q_c;
  }//下面則是一裡一外的狀況
  PVector ray = PVector.sub(p, x0);
  // x = x0.x+d*ray.x, y = x0.y+d*ray.y, z = x0.z+d*ray.z 直線方程式(x0 + d * ray)
  // (x-sphere.x)^2 + (y-sphere.y)^2 + (z-sphere.z)^2 = 100^2 把點代入圓球表面方程式
  // (x0.x+d*ray.x-sphere.x)^2 + (...)^2 + (...)^2 - 100000 = 0 努力展開
  // d^2 * (...) + d * (...) + (...) = 0 把 d^2, d^1, d^0 項分開
  // d = (-b+- sqrt(b*b-4*a*c) ) /(2*a) 套用一元二次方程式 的公式解
  float a = ray.x*ray.x + ray.y*ray.y + ray.z*ray.z;
  float b = 2*ray.x*(x0.x-sphere.x) + 2*ray.y*(x0.y-sphere.y) + 2*ray.z*(x0.z-sphere.z);
  float c = (x0.x-sphere.x)*(x0.x-sphere.x) + (x0.y-sphere.y)*(x0.y-sphere.y) + (x0.z-sphere.z)*(x0.z-sphere.z) - 10000;
  float inside = b*b-4*a*c; //這是要開根號的部分, 應該要大於0。但如果小於0, 那就無法解, 改找 p最近的圓球表面
  if(inside<0){
    print("出錯了出錯了");
    PVector q_c = PVector.add( sphere, PVector.sub(p, sphere).normalize().mult(100) );//找p最近的圓球表面
    return q_c;
  }
  print("="); //continuous collision
  float d1 = (-b + sqrt(inside)) / (2*a);
  float d2 = (-b - sqrt(inside)) / (2*a);
  if( abs(d1) > abs(d2) ) return PVector.add(x0, PVector.mult(ray,d2));
  else return PVector.add(x0, PVector.mult(ray,d1));
}
void projectConstraints(float k2){
  //下面用不同的迴圈寫法,來檢查不同順序的結果
  for( Stick s : sticks ){ //模擬時,可能因順序問題,結果不對稱
    Particle p1 = s.p1, p2 = s.p2;
    float C = C(s);
    PVector n = PVector.sub(p1.p, p2.p).normalize();
    PVector dp1 = PVector.mult(n, -p1.w/(p1.w+p2.w)*C*k2);
    PVector dp2 = PVector.mult(n, +p2.w/(p1.w+p2.w)*C*k2);
    p1.p.add(dp1);
    p2.p.add(dp2);
  }
}
void generateCloth(float w, float h, int wr, int hr){
  sticks = new ArrayList<Stick>();
  particles = new ArrayList<Particle>();
  triangles = new ArrayList<Triangle>();
  for(int i=0; i<=hr; i++){ //build particles
    for(int j=0; j<=wr; j++){
      if(i%2==0){ //偶數排, 會有 wr個間隔, wr+1個頂點
        particles.add( new Particle( (j-wr/2.0)*w/wr, (i-hr/2.0)*h/hr ) );
      }else{ //奇數排, 最左邊有1個小間隔, 最右邊有個小間隔, 中間有 wr-1個間隔, 共計 wr+2個頂點
        if(j==0) particles.add( new Particle( (0-wr/2.0)*w/wr,  (i-hr/2.0)*h/hr ) ); //(多出1個)最左邊對齊的點
        
        if(j==wr) particles.add( new Particle( (wr-wr/2.0)*w/wr,  (i-hr/2.0)*h/hr ) ); //最右邊對齊的點
        else particles.add( new Particle( (j-wr/2.0+0.5)*w/wr, (i-hr/2.0)*h/hr ) );
      }
    }
  } //build particles
  int bottom_row_begin = 0, top_row_begin = 0;
  for(int i=0; i<hr; i++){ //build triangles and sticks
    top_row_begin = bottom_row_begin; //(i/2)*(wr+1+wr+2);
    if(i%2==0) bottom_row_begin = top_row_begin + wr + 1;
    else bottom_row_begin = top_row_begin + wr + 2;
    for(int j=0; j<=wr; j++){
      if(i%2==0){ //偶數排
        if(j==0){ //最左邊的小三角形
          //Q: 要怎麼加 sticks 又不重覆呢?
          triangles.add( new Triangle(particles.get(top_row_begin), particles.get(bottom_row_begin), particles.get(bottom_row_begin+1)) );
          sticks.add( new Stick(particles.get(top_row_begin), particles.get(bottom_row_begin)) );
          sticks.add( new Stick(particles.get(top_row_begin+wr), particles.get(bottom_row_begin+wr+1)) );
        }else{ //兩個一組的反、正三角形
          triangles.add( new Triangle(particles.get(top_row_begin+j), particles.get(top_row_begin+j-1), particles.get(bottom_row_begin+j)) ); 
          triangles.add( new Triangle(particles.get(top_row_begin+j), particles.get(bottom_row_begin+j), particles.get(bottom_row_begin+j+1)) ); 
          sticks.add( new Stick(particles.get(top_row_begin+j), particles.get(top_row_begin+j-1)) );
          sticks.add( new Stick(particles.get(top_row_begin+j), particles.get(bottom_row_begin+j)) );
          sticks.add( new Stick(particles.get(bottom_row_begin+j), particles.get(top_row_begin+j-1)) );
        }
      }else{ //奇數排
        if(j==wr){ //最右邊的小三角形,要多做一個
          triangles.add( new Triangle(particles.get(top_row_begin+wr+1), particles.get(top_row_begin+wr), particles.get(bottom_row_begin+wr)) );
          sticks.add( new Stick(particles.get(top_row_begin+wr+1), particles.get(top_row_begin+wr)) );
          sticks.add( new Stick(particles.get(top_row_begin+wr+1), particles.get(bottom_row_begin+wr)) );
          sticks.add( new Stick(particles.get(bottom_row_begin+wr), particles.get(top_row_begin+wr)) );
        }else{//兩個一組的反、正三角形
          triangles.add( new Triangle(particles.get(top_row_begin+j+1), particles.get(top_row_begin+j), particles.get(bottom_row_begin+j)) );
          triangles.add( new Triangle(particles.get(top_row_begin+j+1), particles.get(bottom_row_begin+j), particles.get(bottom_row_begin+j+1)) ); 
          sticks.add( new Stick(particles.get(top_row_begin+j+1), particles.get(top_row_begin+j)) );
          sticks.add( new Stick(particles.get(top_row_begin+j+1), particles.get(bottom_row_begin+j)) );
          sticks.add( new Stick(particles.get(bottom_row_begin+j), particles.get(top_row_begin+j)) );
          
        }
      } //奇數排
    } //for j
  } //for i, build triangles and sticks
  for(int j=0; j<wr; j++){ //last row sticks
    sticks.add( new Stick(particles.get(bottom_row_begin+j), particles.get(bottom_row_begin+j+1)) );
  }
  //hr為奇數時, 右下角會漏1個棒子, 要補上
  if(hr%2==1) sticks.add( new Stick(particles.get(bottom_row_begin+wr), particles.get(bottom_row_begin+wr+1)) );

  particles.get(0).locked=true; particles.get(0).w=0;
  particles.get(wr).locked=true; particles.get(wr).w=0;
}

float C(Stick s){
  Particle p1 = s.p1, p2 = s.p2;
  return PVector.dist(p1.p, p2.p) - s.len; //先將d設成40
}
void mouseDragged(){ //讓布料的左上角,可以隨著 mouseDragged()移動
  Particle p = particles.get(0);
  p.p.x = p.x.x=mouseX-width/2;
  p.p.y = p.x.y=mouseY-height/2;
}
void mouseWheel(MouseEvent event){
  float e = event.getCount();
  sphere.z += e; //利用 mouse wheel 滾輪 調整大班的圓心, 以便測試 Collision
}

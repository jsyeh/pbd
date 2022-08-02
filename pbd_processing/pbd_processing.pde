//My implementation of Position Based Dynamics
//https://matthias-research.github.io/pages/publications/posBasedDyn.pdf
class Particle{
  PVector x; //position
  PVector p; //estimated position in simulation
  PVector v; //velocity
  boolean locked;
  Particle( float _x, float _y, float _z, float _vx, float _vy, float _vz ){
    x = new PVector(_x, _y, _z);
    p = new PVector(_x, _y, _z);
    v = new PVector(_vx, _vy, _vz);
  }
  Particle( float _x, float _y, float _vx, float _vy ){
    this( _x, _y, 0.0, _vx, _vy, 0.0);
  }
  Particle( float _x, float _y ){
    this( _x, _y, 0.0, 0.0 );
  }
}
class Stick{
  Particle p1, p2;
  Stick( Particle _p1, Particle _p2 ){
    p1 = _p1;
    p2 = _p2;
  }
}
class Triangle{
  Particle [] p = new Particle[3];
  //Stick [] s = new Stick[3];
  Triangle( Particle _p0, Particle _p1, Particle _p2 ){
    p[0] = _p0;
    p[1] = _p1;
    p[2] = _p2;
  }
}
ArrayList<Stick> sticks;
ArrayList<Particle> particles;
ArrayList<Triangle> triangles;
void setup(){
  size(500,500);
  generateCloth(400, 400, 10, 10);
}
void draw(){
  background(#FFFFF2);
  translate(width/2, height/2);
  for( Triangle t : triangles ){
    PVector p0 = t.p[0].x, p1 = t.p[1].x, p2 = t.p[2].x;
    fill(255, 0, 0, 128); stroke(0);
    triangle( p0.x, p0.y, p1.x, p1.y, p2.x, p2.y );
  }
  for( Stick s : sticks ){
    PVector p1 = s.p1.x, p2=s.p2.x;
    stroke(255,0,0);
    line( p1.x, p1.y, p2.x, p2.y );
  }
  for( Particle p : particles ){
    PVector pt = p.x;
    if(p.locked) fill(#FF0000);
    else noFill();
    stroke(0);
    ellipse( pt.x, pt.y, 3, 3 );
  }
  Simulation();
}
void Simulation(){
  PVector gravity = new PVector(0, 0.98, 0);
  for( Particle p : particles ){ //update external force
    p.v.add( gravity ); //(5) vi += dt*wi*fext(xi)
    p.v.mult(0); //先清為零,才不會往下掉//(6) dampVelocities()
    p.p = PVector.add(p.x, p.v); //(7) pi = xi + dt*vi
 }
  //(8) generateCollisionConstraints()
  for(int k=0; k<20; k++){ //(9) solverIterations
    projectConstraints(); //(10) projectConstraints() in Gauss-Seidel fashion
  }
  for( Particle p : particles ){//(12) forall vertices i
    p.v = PVector.sub(p.p, p.x); //(13) vi = (pi-xi)/dt
    p.x.x = p.p.x; //(14) xi = pi
    p.x.y = p.p.y;
    p.x.z = p.p.z;
  }
  //(16) velocityUpdate()
}
void projectConstraints(){
  
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
}

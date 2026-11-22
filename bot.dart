// Narrow the scan around a contact and lead shots from recent sightings.
// Clunkyard guide: https://clunkyard.com/sdk/dart
import 'dart:math' as math;
import 'package:clunkyard/clunkyard.dart';

const searchArc=40.0;  // degrees: sweeps the circle in half a second
const holdWidth=60.0;  // units across at the target: it stays in the beam from tick to tick
const minArc=6.0;      // degrees: the narrowest the beam holds
const window=12;       // ticks of sightings the aim averages
const sightings=8;     // narrow sightings in the window before a shot
const lookEvery=40;    // ticks between looks around the target, for a nearer robot
const reacquire=10;    // ticks to look again where a lost target was

class Bot extends Robot {
  bool free=false;int dodges=0,dodgeEnd=0,slot=0;
  double steer(double towards,double current)=>angles.clamp(angles.angleDiff(towards,current)/45);
  @override void onStart(){free=match.is3d&&arena.gravity.z==0;}
  // A straight line through the narrowest of the last [window] ticks' sightings:
  // where the robot is now, how fast it moves, and how many sightings it rests
  // on. Each sighting is off by its bearingError; the line averages that out.
  (Vec3,Vec3,int)? fit(int slot,int tick){
    final recent=tracker.history(slot).where((s)=>s.tick>tick-window).toList();
    if(recent.length<2)return null;
    final best=recent.map((s)=>s.bearingError).reduce(math.min);
    final seen=recent.where((s)=>s.bearingError<=1.5*best).toList(),n=seen.length;
    if(n<2)return null;
    final t0=seen.fold(0.0,(sum,s)=>sum+s.tick)/n;
    final spread=seen.fold(0.0,(sum,s)=>sum+(s.tick-t0)*(s.tick-t0));
    final position=<double>[],velocity=<double>[];
    for(final axis in [(Vec3 v)=>v.x,(Vec3 v)=>v.y,(Vec3 v)=>v.z]){
      final mean=seen.fold(0.0,(sum,s)=>sum+axis(s.position))/n;
      final slope=seen.fold(0.0,(sum,s)=>sum+(s.tick-t0)*(axis(s.position)-mean))/spread;
      position.add(mean+slope*(tick-t0));velocity.add(slope*match.tickRate);
    }
    return (Vec3(position[0],position[1],position[2]),Vec3(velocity[0],velocity[1],velocity[2]),n);
  }
  @override void onTurn(){
    final self=me,position=me.position,tick=clock.tick;
    drive.forces(brake:0.2);
    final target=sensors.nearest,cue=tracker.cue;
    final lastSeen=slot!=0?tracker.lastSeen(slot):null;
    if(target!=null){
      slot=target.slot;
      final throttle=target.range>350?0.7:target.range<200?-0.4:0.2;
      drive.set(throttle:throttle,turn:steer(target.bearing,self.heading),pitchTurn:steer(target.elevation,self.pitch));
      // Halve the beam each tick until it is holdWidth across at the target:
      // narrow enough for a precise bearing, wide enough to keep it in.
      final hold=math.max(minArc,2*math.atan2(holdWidth/2,target.range)*180/math.pi);
      // In a crowd a narrow beam sees one robot. Every lookEvery ticks,
      // widen it for a tick: a nearer robot it finds becomes the target.
      final arc=tick%lookEvery==0?searchArc:math.max(hold,math.min(searchArc,scanner.last.arc/2));
      scanner.set(heading:target.bearing,arc:arc,pitch:target.elevation);
      final power=gun.powerForFlightTime(target.range,1);
      // An exact contact: gun.lead is the intercept if the target holds
      // course. A noisy one: lead the fitted line, and fire only once it
      // rests on enough sightings.
      Solution? aim;var ready=true;
      if(target.bearingError>0){
        final line=fit(target.slot,tick);
        aim=line==null?null:gun.leadPoint(line.$1,line.$2,power);
        ready=line!=null&&line.$3>=sightings;
      }else{aim=gun.lead(target,power);}
      final heading=aim?.heading??target.bearing,pitch=aim?.pitch??target.elevation;
      gun.aim(heading:heading,pitch:pitch);final now=gun.aimNow;
      if(ready&&angles.angleDiff(heading,now.heading).abs()<3&&(pitch-now.pitch).abs()<3&&gun.canFire(power,10))gun.fire(power:power);
      status.set(text:ready?'TRACK':'NARROW',value:target.range);
    }else if(lastSeen!=null&&tick-lastSeen<=reacquire){
      // Lost it: look again where it was last seen, widening the beam, before searching.
      final where=tracker.position(slot)!;
      final bearing=angles.bearingTo(position,where),elevation=free?angles.elevationTo(position,where):0.0;
      drive.set(throttle:0.5,turn:steer(bearing,self.heading),pitchTurn:steer(elevation,self.pitch));
      scanner.set(heading:bearing,arc:math.min(searchArc,scanner.last.arc*2),pitch:elevation);status.set(text:'REACQUIRE');
    }else if(cue!=null){
      final bearing=angles.bearingTo(position,cue.position),distance=angles.distance(position,cue.position);
      final elevation=free?angles.elevationTo(position,cue.position):0.0,error=math.min(cue.bearingError,30);
      final across=(tick%5-2)/2*error,up=free?((tick~/5)%3-1)*error/2:0.0;
      drive.set(throttle:0.7,turn:steer(bearing,self.heading),pitchTurn:steer(elevation,self.pitch));
      scanner.set(heading:bearing+across,arc:scanner.arcForReach(distance+100),pitch:elevation+up);status.set(text:'CUE',value:distance);
    }else{
      drive.set(throttle:0.5,turn:0.33,pitchTurn:steer(0,self.pitch));
      final pitch=free?((tick~/10)%3-1)*35.0:0.0;
      scanner.set(heading:angles.normalize(scanner.last.heading+36),arc:searchArc,pitch:pitch);status.set(text:'SEARCH');
    }
    // A simple dodge: vary it, or predict the shooter.
    if(tick>=dodgeEnd&&sensors.cues.any((c)=>c.source==Source.muzzle)){dodges++;dodgeEnd=tick+10;}
    if(tick<dodgeEnd){
      // Off the line of fire: left, then right next time (turn is positive left, strafe right).
      final left=dodges.isOdd;
      if(match.control==Control.forces){drive.set(throttle:0.5);drive.forces(strafe:left?-1:1);}
      else drive.set(throttle:1,turn:left?1:-1);
      status.set(text:left?'dodge:left':'dodge:right',value:dodges.toDouble());
    }
    if(battery.charge<battery.capacity/4)drive.set(throttle:(drive.get('throttle')??0)*0.5);
    final storm=nav.storm,zone=nav.zone;
    if(storm!=null&&(storm.margin(position)??-1e9)>-60){
      final away=angles.bearingTo(storm.center,position);
      drive.set(throttle:0.8,turn:angles.clamp(angles.angleDiff(away,self.heading)/30));
    }
    if(zone!=null&&(zone.margin(position)??1e9)<60){
      final climb=zone.shape==Shape.sphere?angles.elevationTo(position,zone.center):0.0;
      drive.set(throttle:0.8,turn:angles.clamp(angles.angleDiff(angles.bearingTo(position,zone.center),self.heading)/30),pitchTurn:angles.clamp((climb-self.pitch)/30));
    }
  }
}


void main()=>run(Bot());

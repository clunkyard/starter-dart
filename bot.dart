// A robot on the core binding supplied by the arena.
// Protocol parsing, tracking, geometry and act serialization live in the core.
// Clunkyard robot API 2.0.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude applied
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range)
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   tracker.configure(smoothing forget_ticks cue_ticks)
//   status.set(text value) raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
import 'dart:math' as math;
import 'package:clunkyard/clunkyard.dart';

class Bot extends Robot {
  bool free=false;
  double steer(double towards,double current)=>angles.clamp(angles.angleDiff(towards,current)/45);
  @override void onStart(){free=match.is3d&&arena.gravity.z==0;}
  @override void onTurn(){
    final self=me,position=me.position;
    drive.forces(brake:0.2);
    final target=sensors.nearest,cue=tracker.cue;
    if(target!=null){
      final throttle=target.range>350?0.7:target.range<200?-0.4:0.2;
      drive.set(throttle:throttle,turn:steer(target.bearing,self.heading),pitchTurn:steer(target.elevation,self.pitch));
      scanner.set(heading:target.bearing,arc:20,pitch:target.elevation);
      final power=gun.powerForFlightTime(target.range,1),aim=gun.lead(target,power);
      final heading=aim?.heading??target.bearing,pitch=aim?.pitch??target.elevation;
      gun.aim(heading:heading,pitch:pitch);final now=gun.aimNow;
      if(angles.angleDiff(heading,now.heading).abs()<3&&(pitch-now.pitch).abs()<3&&gun.canFire(power,10))gun.fire(power:power);
      status.set(text:'TRACK',value:target.range);
    }else if(cue!=null){
      final bearing=angles.bearingTo(position,cue.position),distance=angles.distance(position,cue.position);
      final elevation=free?angles.elevationTo(position,cue.position):0.0,error=math.min(cue.bearingError,30),tick=clock.tick;
      final across=(tick%5-2)/2*error,up=free?((tick~/5)%3-1)*error/2:0.0;
      drive.set(throttle:0.7,turn:steer(bearing,self.heading),pitchTurn:steer(elevation,self.pitch));
      scanner.set(heading:bearing+across,arc:scanner.arcForReach(distance+100),pitch:elevation+up);status.set(text:'CUE',value:distance);
    }else{
      drive.set(throttle:0.5,turn:0.33,pitchTurn:steer(0,self.pitch));
      final pitch=free?((clock.tick~/10)%3-1)*35.0:0.0;
      scanner.set(heading:angles.normalize(scanner.last.heading+36),arc:40,pitch:pitch);status.set(text:'SEARCH');
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

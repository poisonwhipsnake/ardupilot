/*
 * Location.cpp
 */

#include "Location.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Terrain/AP_Terrain.h>

/// constructors
Location::Location()
{
    zero();
}

const Location definitely_zero{};
bool Location::is_zero(void) const
{
    return !memcmp(this, &definitely_zero, sizeof(*this));
}

void Location::zero(void)
{
    memset(this, 0, sizeof(*this));
}

// Construct location using position (NEU) from ekf_origin for the given altitude frame
Location::Location(int32_t latitude, int32_t longitude, int32_t alt_in_cm, AltFrame frame)
{
    // make sure we know what size the Location object is:
    ASSERT_STORAGE_SIZE(Location, 16);

    zero();
    lat = latitude;
    lng = longitude;
    set_alt_cm(alt_in_cm, frame);
}


Location::Location(const Vector3f &ekf_offset_neu, AltFrame frame)
{
    zero();

    // store alt and alt frame
    set_alt_cm(ekf_offset_neu.z, frame);

    // calculate lat, lon
    Location ekf_origin;
    if (AP::ahrs().get_origin(ekf_origin)) {
        lat = ekf_origin.lat;
        lng = ekf_origin.lng;
        offset(ekf_offset_neu.x * 0.01, ekf_offset_neu.y * 0.01);
    }
}

void Location::clone(const Location &clone){
    lat = clone.lat;
    lng = clone.lng;
    alt = clone.alt;
    relative_alt = clone.relative_alt;           // 1 if altitude is relative to home
    loiter_ccw   =clone.loiter_ccw;           // 0 if clockwise, 1 if counter clockwise
    terrain_alt  = clone.terrain_alt;           // this altitude is above terrain
    origin_alt   = clone.origin_alt;           // this altitude is above ekf origin
    loiter_xtrack = clone.loiter_xtrack;          // 0 to crosstrack from center of waypoint, 1 to crosstrack from tangent exit location
}



void Location::set_alt_cm(int32_t alt_cm, AltFrame frame)
{
    alt = alt_cm;
    relative_alt = false;
    terrain_alt = false;
    origin_alt = false;
    switch (frame) {
        case AltFrame::ABSOLUTE:
            // do nothing
            break;
        case AltFrame::ABOVE_HOME:
            relative_alt = true;
            break;
        case AltFrame::ABOVE_ORIGIN:
            origin_alt = true;
            break;
        case AltFrame::ABOVE_TERRAIN:
            // we mark it as a relative altitude, as it doesn't have
            // home alt added
            relative_alt = true;
            terrain_alt = true;
            break;
    }
}

// converts altitude to new frame
bool Location::change_alt_frame(AltFrame desired_frame)
{
    int32_t new_alt_cm;
    if (!get_alt_cm(desired_frame, new_alt_cm)) {
        return false;
    }
    set_alt_cm(new_alt_cm, desired_frame);
    return true;
}

// get altitude frame
Location::AltFrame Location::get_alt_frame() const
{
    if (terrain_alt) {
        return AltFrame::ABOVE_TERRAIN;
    }
    if (origin_alt) {
        return AltFrame::ABOVE_ORIGIN;
    }
    if (relative_alt) {
        return AltFrame::ABOVE_HOME;
    }
    return AltFrame::ABSOLUTE;
}

/// get altitude in desired frame
bool Location::get_alt_cm(AltFrame desired_frame, int32_t &ret_alt_cm) const
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (!initialised()) {
        AP_HAL::panic("Should not be called on invalid location: Location cannot be (0, 0, 0)");
    }
#endif
    Location::AltFrame frame = get_alt_frame();

    // shortcut if desired and underlying frame are the same
    if (desired_frame == frame) {
        ret_alt_cm = alt;
        return true;
    }

    // check for terrain altitude
    float alt_terr_cm = 0;
    if (frame == AltFrame::ABOVE_TERRAIN || desired_frame == AltFrame::ABOVE_TERRAIN) {
#if AP_TERRAIN_AVAILABLE
        AP_Terrain *terrain = AP::terrain();
        if (terrain == nullptr) {
            return false;
        }
        if (!terrain->height_amsl(*this, alt_terr_cm)) {
            return false;
        }
        // convert terrain alt to cm
        alt_terr_cm *= 100.0f;
#else
        return false;
#endif
    }

    // convert alt to absolute
    int32_t alt_abs = 0;
    switch (frame) {
        case AltFrame::ABSOLUTE:
            alt_abs = alt;
            break;
        case AltFrame::ABOVE_HOME:
#if AP_AHRS_ENABLED
            if (!AP::ahrs().home_is_set()) {
                return false;
            }
            alt_abs = alt + AP::ahrs().get_home().alt;
#else
            return false;
#endif  // AP_AHRS_ENABLED
            break;
        case AltFrame::ABOVE_ORIGIN:
#if AP_AHRS_ENABLED
            {
                // fail if we cannot get ekf origin
                Location ekf_origin;
                if (!AP::ahrs().get_origin(ekf_origin)) {
                    return false;
                }
                alt_abs = alt + ekf_origin.alt;
            }
            break;
#else
            return false;
#endif  // AP_AHRS_ENABLED
        case AltFrame::ABOVE_TERRAIN:
            alt_abs = alt + alt_terr_cm;
            break;
    }

    // convert absolute to desired frame
    switch (desired_frame) {
        case AltFrame::ABSOLUTE:
            ret_alt_cm = alt_abs;
            return true;
        case AltFrame::ABOVE_HOME:
#if AP_AHRS_ENABLED
            if (!AP::ahrs().home_is_set()) {
                return false;
            }
            ret_alt_cm = alt_abs - AP::ahrs().get_home().alt;
#else
            return false;
#endif  // AP_AHRS_ENABLED
            return true;
        case AltFrame::ABOVE_ORIGIN:
#if AP_AHRS_ENABLED
            {
                // fail if we cannot get ekf origin
                Location ekf_origin;
                if (!AP::ahrs().get_origin(ekf_origin)) {
                    return false;
                }
                ret_alt_cm = alt_abs - ekf_origin.alt;
                return true;
            }
#else
            return false;
#endif  // AP_AHRS_ENABLED
        case AltFrame::ABOVE_TERRAIN:
            ret_alt_cm = alt_abs - alt_terr_cm;
            return true;
    }
    return false;  // LCOV_EXCL_LINE  - not reachable
}

#if AP_AHRS_ENABLED
bool Location::get_vector_xy_from_origin_NE(Vector2f &vec_ne) const
{
    Location ekf_origin;
    if (!AP::ahrs().get_origin(ekf_origin)) {
        return false;
    }
    vec_ne.x = (lat-ekf_origin.lat) * LATLON_TO_CM;
    vec_ne.y = diff_longitude(lng,ekf_origin.lng) * LATLON_TO_CM * longitude_scale((lat+ekf_origin.lat)/2);
    return true;
}

bool Location::get_vector_from_origin_NEU(Vector3f &vec_neu) const
{
    // convert lat, lon
    if (!get_vector_xy_from_origin_NE(vec_neu.xy())) {
        return false;
    }

    // convert altitude
    int32_t alt_above_origin_cm = 0;
    if (!get_alt_cm(AltFrame::ABOVE_ORIGIN, alt_above_origin_cm)) {
        return false;
    }
    vec_neu.z = alt_above_origin_cm;

    return true;
}
#endif  // AP_AHRS_ENABLED

// return horizontal distance in meters between two locations
ftype Location::get_distance(const Location &loc2) const
{
    ftype dlat = (ftype)(loc2.lat - lat);
    ftype dlng = ((ftype)diff_longitude(loc2.lng,lng)) * longitude_scale((lat+loc2.lat)/2);
    return norm(dlat, dlng) * LOCATION_SCALING_FACTOR;
}

// return the altitude difference in meters taking into account alt frame.
bool Location::get_alt_distance(const Location &loc2, ftype &distance) const
{
    int32_t alt1, alt2;
    if (!get_alt_cm(AltFrame::ABSOLUTE, alt1) || !loc2.get_alt_cm(AltFrame::ABSOLUTE, alt2)) {
        return false;
    }
    distance = (alt1 - alt2) * 0.01;
    return true;
}

/*
  return the distance in meters in North/East plane as a N/E vector
  from loc1 to loc2
 */
Vector2f Location::get_distance_NE(const Location &loc2) const
{
    return Vector2f((loc2.lat - lat) * LOCATION_SCALING_FACTOR,
                    diff_longitude(loc2.lng,lng) * LOCATION_SCALING_FACTOR * longitude_scale((loc2.lat+lat)/2));
}

// return the distance in meters in North/East/Down plane as a N/E/D vector to loc2, NOT CONSIDERING ALT FRAME!
Vector3f Location::get_distance_NED(const Location &loc2) const
{
    return Vector3f((loc2.lat - lat) * LOCATION_SCALING_FACTOR,
                    diff_longitude(loc2.lng,lng) * LOCATION_SCALING_FACTOR * longitude_scale((lat+loc2.lat)/2),
                    (alt - loc2.alt) * 0.01);
}

// return the distance in meters in North/East/Down plane as a N/E/D vector to loc2
Vector3d Location::get_distance_NED_double(const Location &loc2) const
{
    return Vector3d((loc2.lat - lat) * double(LOCATION_SCALING_FACTOR),
                    diff_longitude(loc2.lng,lng) * LOCATION_SCALING_FACTOR * longitude_scale((lat+loc2.lat)/2),
                    (alt - loc2.alt) * 0.01);
}

Vector2d Location::get_distance_NE_double(const Location &loc2) const
{
    return Vector2d((loc2.lat - lat) * double(LOCATION_SCALING_FACTOR),
                    diff_longitude(loc2.lng,lng) * double(LOCATION_SCALING_FACTOR) * longitude_scale((lat+loc2.lat)/2));
}

Vector2F Location::get_distance_NE_ftype(const Location &loc2) const
{
    return Vector2F((loc2.lat - lat) * ftype(LOCATION_SCALING_FACTOR),
                    diff_longitude(loc2.lng,lng) * ftype(LOCATION_SCALING_FACTOR) * longitude_scale((lat+loc2.lat)/2));
}

// extrapolate latitude/longitude given distances (in meters) north and east
void Location::offset_latlng(int32_t &lat, int32_t &lng, ftype ofs_north, ftype ofs_east)
{
    const int32_t dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
    const int64_t dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale(lat+dlat/2);
    lat += dlat;
    lat = limit_lattitude(lat);
    lng = wrap_longitude(dlng+lng);
}

// extrapolate latitude/longitude given distances (in meters) north and east
void Location::offset(ftype ofs_north, ftype ofs_east)
{
    offset_latlng(lat, lng, ofs_north, ofs_east);
}

// extrapolate latitude/longitude given distances (in meters) north
// and east. Note that this is metres, *even for the altitude*.
void Location::offset(const Vector3p &ofs_ned)
{
    offset_latlng(lat, lng, ofs_ned.x, ofs_ned.y);
    alt += -ofs_ned.z * 100;  // m -> cm
}

/*
 *  extrapolate latitude/longitude given bearing and distance
 * Note that this function is accurate to about 1mm at a distance of
 * 100m. This function has the advantage that it works in relative
 * positions, so it keeps the accuracy even when dealing with small
 * distances and floating point numbers
 */
void Location::offset_bearing(ftype bearing_deg, ftype distance)
{
    const ftype ofs_north = cosF(radians(bearing_deg)) * distance;
    const ftype ofs_east  = sinF(radians(bearing_deg)) * distance;
    offset(ofs_north, ofs_east);
}

// extrapolate latitude/longitude given bearing, pitch and distance
void Location::offset_bearing_and_pitch(ftype bearing_deg, ftype pitch_deg, ftype distance)
{
    const ftype ofs_north =  cosF(radians(pitch_deg)) * cosF(radians(bearing_deg)) * distance;
    const ftype ofs_east  =  cosF(radians(pitch_deg)) * sinF(radians(bearing_deg)) * distance;
    offset(ofs_north, ofs_east);
    const int32_t dalt =  sinF(radians(pitch_deg)) * distance *100.0f;
    alt += dalt; 
}


ftype Location::longitude_scale(int32_t lat)
{
    ftype scale = cosF(lat * (1.0e-7 * DEG_TO_RAD));
    return MAX(scale, 0.01);
}

/*
 * convert invalid waypoint with useful data. return true if location changed
 */
bool Location::sanitize(const Location &defaultLoc)
{
    bool has_changed = false;
    // convert lat/lng=0 to mean current point
    if (lat == 0 && lng == 0) {
        lat = defaultLoc.lat;
        lng = defaultLoc.lng;
        has_changed = true;
    }

    // convert relative alt=0 to mean current alt
    if (alt == 0 && relative_alt) {
        int32_t defaultLoc_alt;
        if (defaultLoc.get_alt_cm(get_alt_frame(), defaultLoc_alt)) {
            alt = defaultLoc_alt;
            has_changed = true;
        }
    }

    // limit lat/lng to appropriate ranges
    if (!check_latlng()) {
        lat = defaultLoc.lat;
        lng = defaultLoc.lng;
        has_changed = true;
    }

    return has_changed;
}

// return bearing in radians from location to loc2, return is 0 to 2*Pi
ftype Location::get_bearing(const Location &loc2) const
{
    const int32_t off_x = diff_longitude(loc2.lng,lng);
    const int32_t off_y = (loc2.lat - lat) / loc2.longitude_scale((lat+loc2.lat)/2);
    ftype bearing = (M_PI*0.5) + atan2F(-off_y, off_x);
    if (bearing < 0) {
        bearing += 2*M_PI;
    }
    return bearing;
}

/*
  return true if lat and lng match. Ignores altitude and options
 */
bool Location::same_latlon_as(const Location &loc2) const
{
    return (lat == loc2.lat) && (lng == loc2.lng);
}

bool Location::same_alt_as(const Location &loc2) const
{
    // fast path if the altitude frame is the same
    if (this->get_alt_frame() == loc2.get_alt_frame()) {
        return this->alt == loc2.alt;
    }

    ftype alt_diff;
    bool have_diff = this->get_alt_distance(loc2, alt_diff);

    const ftype tolerance = FLT_EPSILON;
    return have_diff && (fabsF(alt_diff) < tolerance);
}

// return true when lat and lng are within range
bool Location::check_latlng() const
{
    return check_lat(lat) && check_lng(lng);
}

// see if location is past a line perpendicular to
// the line between point1 and point2 and passing through point2.
// If point1 is our previous waypoint and point2 is our target waypoint
// then this function returns true if we have flown past
// the target waypoint
bool Location::past_interval_finish_line(const Location &point1, const Location &point2) const
{
    return this->line_path_proportion(point1, point2) >= 1.0f;
}

/*
  return the proportion we are along the path from point1 to
  point2, along a line parallel to point1<->point2.

  This will be more than 1 if we have passed point2
 */
float Location::line_path_proportion(const Location &point1, const Location &point2) const
{
    const Vector2f vec1 = point1.get_distance_NE(point2);
    const Vector2f vec2 = point1.get_distance_NE(*this);
    const ftype dsquared = sq(vec1.x) + sq(vec1.y);
    if (dsquared < 0.001f) {
        // the two points are very close together
        return 1.0f;
    }
    return (vec1 * vec2) / dsquared;
}


/*
  wrap longitude for -180e7 to 180e7
 */
int32_t Location::wrap_longitude(int64_t lon)
{
    if (lon > 1800000000L) {
        lon = int32_t(lon-3600000000LL);
    } else if (lon < -1800000000L) {
        lon = int32_t(lon+3600000000LL);
    }
    return int32_t(lon);
}

/*
  get lon1-lon2, wrapping at -180e7 to 180e7
 */
int32_t Location::diff_longitude(int32_t lon1, int32_t lon2)
{
    if ((lon1 & 0x80000000) == (lon2 & 0x80000000)) {
        // common case of same sign
        return lon1 - lon2;
    }
    int64_t dlon = int64_t(lon1)-int64_t(lon2);
    if (dlon > 1800000000LL) {
        dlon -= 3600000000LL;
    } else if (dlon < -1800000000LL) {
        dlon += 3600000000LL;
    }
    return int32_t(dlon);
}

/*
  limit latitude to -90e7 to 90e7
 */
int32_t Location::limit_lattitude(int32_t lat)
{
    if (lat > 900000000L) {
        lat = 1800000000LL - lat;
    } else if (lat < -900000000L) {
        lat = -(1800000000LL + lat);
    }
    return lat;
}

// update altitude and alt-frame base on this location's horizontal position between point1 and point2
// this location's lat,lon is used to calculate the alt of the closest point on the line between point1 and point2
// origin and destination's altitude frames must be the same
// this alt-frame will be updated to match the destination alt frame
void Location::linearly_interpolate_alt(const Location &point1, const Location &point2)
{
    // new target's distance along the original track and then linear interpolate between the original origin and destination altitudes
    set_alt_cm(point1.alt + (point2.alt - point1.alt) * constrain_float(line_path_proportion(point1, point2), 0.0f, 1.0f), point2.get_alt_frame());
}


void Location::calc_orbit_turn_centre(
    const struct Location &previous_wp,
    const struct Location &current_loc,
    const struct Location &turn_WP,
    const struct Location &next_WP,
    float ground_turn_radius,
    AP_Float ground_turn_early_initiation,
    Location &out_turn_centre,
    Vector3f &out_turn_vector,
    int8_t &out_turn_direction,
    float &out_groundspeed_heading_1,
    float &out_groundspeed_heading_2,
    Vector2f &out_turn_start_dist)
{
    // If you have ticked off a waypoint, you are either currently turning on to the track between that ticked off waypoint and the next, or are travelling along that track.
    Vector2f Current_Track_Full = previous_wp.get_distance_NE(turn_WP);     // Linear track between last ticked off WP and next. Is the track to turn onto when completing a turn
    Vector2f Next_Track_Full = turn_WP.get_distance_NE(next_WP);            // Linear track between the next WP and the one after - is used for
    Vector2f Current_Track = Current_Track_Full.normalized();               // Unit vector of "current track"
    Vector2f Next_Track = Next_Track_Full.normalized();                     // Unit vector of "next track"
    Vector3f Current_Vector = Vector3f(Current_Track_Full.x,Current_Track_Full.y,(turn_WP.alt-previous_wp.alt)/100.0f); ////I think this assumes the WP have the same height reference frame
    Vector3f Next_Vector = Vector3f(Next_Track_Full.x,Next_Track_Full.y,(next_WP.alt - turn_WP.alt)/100.0f); ////I think this assumes the WP have the same height reference frame


    float _groundspeed_heading_1 = atan2f(Current_Track.y,Current_Track.x); // The ground-frame heading that is parallel to the "current track" - ideally colinear if the turn tune is working. Used for turn exit calcs.
    float _groundspeed_heading_2 = atan2f(Next_Track.y,Next_Track.x);       // The ground-frame heading that is parallel to the "next track" - ideally colinear if the turn tune is working. Used for turn entry calcs.

    float _ground_turn_angle = wrap_2PI(_groundspeed_heading_2-_groundspeed_heading_1+ M_PI)- M_PI; // The ground-frame angle of the turn ahead (between current and next tracks). Used for turn entry calcs.
    float turn_distance = tanf(abs(_ground_turn_angle/2))*ground_turn_radius;                      // Linear distance away from WP to begin the turn to maintain the parameterised circular ground track of radius 'ground_turn_radius'
    float current_angle_to_track = Current_Track.angle(current_loc.get_distance_NE(turn_WP));       // Assuming tracking with a heading towards 'next WP', find your current track angle compared to the desired 'current_track'. Used for tuning turn entry.


    // Return distance away from WP that the turn should be initiated considering our angle to the desired approach track, the calculated turn distance required, plus an early initiation tunable parameter.
    Vector2f turn_start_dist = (current_loc.get_distance_NE(turn_WP).normalized()/cosf(current_angle_to_track))*(turn_distance+ground_turn_early_initiation);


    int8_t potential_turn_direction = -1;   // Direction of turn, positive is clockwise
    Location potential_turn_centre;
    Vector2f centre_from_turn_WP = -Current_Track*turn_distance + Vector2f(Current_Track.y,-Current_Track.x)*ground_turn_radius;   // Centre of the ground frame turn circle

    // If the ground turn is in the clockwise direction, calculate the centre of ground frame turn for the new case.
    // (This can be consolidated with a rotational matrix with the angle of rotation being sgn(_gta)*pi)
    if(_ground_turn_angle>0){
        centre_from_turn_WP = -Current_Track*turn_distance + Vector2f(-Current_Track.y,Current_Track.x)*ground_turn_radius;
        potential_turn_direction = 1;
    }

    // Generate a z component of the turn via cross product to account for turns that aren't level
    Vector3f potential_turn_vector = (Current_Vector%Next_Vector).normalized();
    // Ensure the z component of the turn is always pointing up
    if(potential_turn_vector.z <0.0f){
        potential_turn_vector = -potential_turn_vector;
    }

    // Unit vector from the turn centre in the direction of the turn WP. Marks the angle of halfway through the turn.
    Vector2f WPFromCentreNormalised = -centre_from_turn_WP.normalized();

    // The desired angle of the normal vector component of the turn. ie. a flat turn will have a desiredAngle of 0deg.
    // A turn where the midpoint is lower than the entry/exit will have a negative desired angle
    float desiredAngle = asinf( Vector2f(potential_turn_vector.x,potential_turn_vector.y)*WPFromCentreNormalised);

    // Height of the WP compared to centre of the planned ground-frame turn (this will be applied as an offset to the ground turn)
    float heightOffset = tanf(desiredAngle)*centre_from_turn_WP.length();

    potential_turn_centre.clone(turn_WP);                                                       // set potential_turn_centre to the WP location
    potential_turn_centre.offset(centre_from_turn_WP.x,centre_from_turn_WP.y);                  // move potential_turn_centre in the N, E plane by the calculated offsets for correct ground frame tracking
    potential_turn_centre.set_alt_cm(turn_WP.alt + (heightOffset*100),turn_WP.get_alt_frame()); // move potential_turn_centre in the Z(D) plane by the height offset by the calculated height offset to ensure we climb/descend in turn

    // Need to return potential turn centre, potential turn direction, ground speed heading 1/2 and return value

    out_turn_centre = potential_turn_centre,
    out_turn_vector = potential_turn_vector,
    out_turn_direction = potential_turn_direction,
    out_groundspeed_heading_1 = _groundspeed_heading_1,
    out_groundspeed_heading_2 = _groundspeed_heading_2,
    out_turn_start_dist = turn_start_dist;

    return;
}

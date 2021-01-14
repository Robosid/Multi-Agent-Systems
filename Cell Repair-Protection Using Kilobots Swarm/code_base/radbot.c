/*
 * Creator: Sid Mahapatra
 * Last Modified: June 04, 2020
 */

#include <math.h>
#include <stdbool.h>
#include <kilombo.h>
#include "radbot.h"

enum {STOP,LEFT,RIGHT,STRAIGHT};

typedef struct
{
  Neighbor_t neighbors[MAXN]; // list of neighbor information
  int N_Neighbors; // number of neighbors
  uint8_t bot_type;
  uint8_t bot_state;
  uint8_t move_type;
  uint8_t RXHead, RXTail;  // used for simulated message processing
  uint16_t my_reading; // own light reading
  uint32_t momentum_check; // time kilobot has been alone
  int own_gradient; // hops from adhesion site kilobot
  int min; // index of neighbor with highest light reading
  bool lock; // flag for adhesion kilobot movement
  bool lost; // flag for no neighbors
  char message_lock; // prevents sending of message
  message_t transmit_msg; // message data object
  received_message_t RXBuffer[RB_SIZE]; // used for simulated message processing
} bot_info;

REGISTER_USERDATA(bot_info)

#ifdef SIMULATOR
#include <stdio.h>    // for printf
#else
#define DEBUG         // for printf to serial port
#include "debug.h"
#endif

// message rx callback function. Pushes message to ring buffer.
void rxbuffer_push(message_t *msg, distance_measurement_t *dist) {
    received_message_t *rmsg = &RB_back();
    rmsg->msg = *msg;
    rmsg->dist = *dist;
    RB_pushback();
}

message_t *message_tx()
{
  if (mydata->message_lock)
    return 0;
  return &mydata->transmit_msg;
}

/* Accessor and mutator functions for bot state and move type
 */
void set_bot_state(int state)
{
  mydata->bot_state = state;
}

int get_bot_state(void)
{
  return mydata->bot_state;
}

void set_move_type(int type)
{
  mydata->move_type = type;
}

int get_move_type(void)
{
  return mydata->move_type;
}

/* Process a received message at the front of the ring buffer.
 * Go through the list of neighbors. If the message is from a bot
 * already in the list, update the information, otherwise
 * add a new entry in the list
*/

void process_message()
{
  mydata->lost = false; // Received a message from neighbors
  uint8_t i;
  uint16_t ID, recv_reading;
  uint8_t *data = RB_front().msg.data;
  // Extract ID from message
  ID = data[0] | (data[1] << 8);
  recv_reading = (data[4] << 8) | (data[5]);
  
  uint8_t d = estimate_distance(&RB_front().dist);

  // Search the neighbor list by ID
  for (i = 0; i < mydata->N_Neighbors; i++)
    if (mydata->neighbors[i].ID == ID)
      {
        // found it
    	break;
      }

  if (i == mydata->N_Neighbors){  // this neighbor is not in list
    if (mydata->N_Neighbors < MAXN-1) // if we have too many neighbors,
      mydata->N_Neighbors++;          // we overwrite the last entry
                                      // sloppy but better than overflow
  }
  // i now points to where this message should be stored
  mydata->neighbors[i].ID = ID;
  mydata->neighbors[i].timestamp = kilo_ticks;
  mydata->neighbors[i].dist = d;
  mydata->neighbors[i].N_Neighbors = data[2];
  mydata->neighbors[i].n_bot_state = data[3];
  mydata->neighbors[i].reading = recv_reading;
  mydata->neighbors[i].ttl = data[7] - 1;
  mydata->neighbors[i].grad = data[6];
}


/* Finds the minimum light reading by comparing all
 * neighbors' readings to own. Returns the index of neighbor
 * with lowest light reading and highest time to live of
 * reading or -1 if this kilobot has the lowest light
 */
int min_reading(){
  int index = -1;
  uint16_t min = mydata->my_reading;
  uint16_t max_ttl = 0;
  for (int i = 0; i < mydata->N_Neighbors; i++){
    if (((mydata->neighbors[i].reading < min) && mydata->neighbors[i].ttl > 0) || ((mydata->neighbors[i].reading <= min) && (mydata->neighbors[i].ttl > max_ttl)))
    {
      max_ttl = mydata->neighbors[i].ttl;
      min = mydata->neighbors[i].reading;
      index = i;
    }
  }
  if (min == mydata->my_reading){
    return -1;
  }
  return index;
}

/* Sets the gradient value of the kilobot by searching
 * through all neighbors finding the lowest gradient
 * and setting own_gradient to that value + 1.
 * Forces gradient to be 0 if the kilobot's own light
 * reading is below a hardcoded threshold (in this case 70)
 */
void set_gradient(){
  int min_grad = mydata->own_gradient;
  if (mydata->my_reading < 70)
    mydata->own_gradient = 0;
  else if (mydata->min != -1){
    for (int i = 0; i < mydata->N_Neighbors; i++){
      if (mydata->neighbors[i].grad < min_grad)
        min_grad = mydata->neighbors[i].grad;
    }
    mydata->own_gradient = min_grad + 1;
  }
  else
    mydata->own_gradient = 0;
}

/* Go through the list of neighbors, remove entries older than a threshold,
 * currently 2 seconds.
 */
void purgeNeighbors(void)
{
  int8_t i;
  for (i = mydata->N_Neighbors-1; i >= 0; i--){
    if (kilo_ticks - mydata->neighbors[i].timestamp  > 64){
      //this one is too old.
      mydata->neighbors[i] = mydata->neighbors[mydata->N_Neighbors-1];
      //replace it by the last entry
      mydata->N_Neighbors--;
    }
  }
}

void setup_message(void)
{
  mydata->message_lock = 1;  //don't transmit while we are forming the message
  mydata->transmit_msg.type = NORMAL;
  mydata->transmit_msg.data[0] = kilo_uid & 0xff;     // 0 low  ID
  mydata->transmit_msg.data[1] = kilo_uid >> 8;       // 1 high ID
  mydata->transmit_msg.data[2] = mydata->N_Neighbors; // 2 number of neighbors
  mydata->transmit_msg.data[3] = get_bot_state();     // 3 bot state
  mydata->my_reading = get_ambientlight();            // record current light
  mydata->min = min_reading();
  if (mydata->min != -1){ // broadcast own light reading with full TTL
    mydata->transmit_msg.data[4] = mydata->neighbors[mydata->min].reading >> 8;
    // 4 low light reading
    mydata->transmit_msg.data[5] = 0 |  mydata->neighbors[mydata->min].reading;
    // 5 high light reading
    mydata->transmit_msg.data[7] = mydata->neighbors[mydata->min].ttl;
    // 7 time to live
    }
  else{ // broadcast neighbor's light reading with their TTL
    mydata->transmit_msg.data[4] = mydata->my_reading >> 8;
    // 4 low light reading
    mydata->transmit_msg.data[5] = 0 | mydata->my_reading;
    // 5 high light reading
    mydata->transmit_msg.data[7] = TTL;
    // 7 time to live
    }
  mydata->transmit_msg.data[6] = mydata->own_gradient; // 6 gradient value
  mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);
  mydata->message_lock = 0;
}

void setup()
{
  rand_seed(kilo_uid + 1); //seed the random number generator
  mydata->message_lock = 0;
  mydata->own_gradient = 0;
  mydata->min = 0;
  mydata->lost = false;
  mydata->my_reading = get_ambientlight();
  mydata->N_Neighbors = 0;
  set_move_type(STOP);
  set_bot_state(LISTEN);
  setup_message();
}

void receive_inputs()
{
  while (!RB_empty())
  {
    process_message();
    RB_popfront();
  }
  purgeNeighbors();
}

/*
 * Returns the distance of the nearest neighbor
 */
uint8_t find_nearest_N_dist()
{
  uint8_t i;
  uint8_t dist = 90;

  for(i = 0; i < mydata->N_Neighbors; i++)
  {
    if(mydata->neighbors[i].dist < dist)
    {
      dist = mydata->neighbors[i].dist;
    }
  }
  return dist;
}

/*
 * Returns the distance of the nearest neighbor
 */
uint8_t find_farthest_N_dist()
{
  uint8_t i;
  uint8_t dist = 0;
  for(i = 0; i < mydata->N_Neighbors; i++)
  {
    if(mydata->neighbors[i].dist > dist)
    {
      dist = mydata->neighbors[i].dist;
    }
  }
  return dist;
}

/*
 * Follows the edge of the 'cell' of kilobots.
 * If too close to any neighbor, the kilobot turns one direction
 * if far enough away, the kilobot turns the opposite direction
 */
void follow_edge()
{
  uint8_t desired_dist = 42;
  set_bot_state(MOVE);
  // currently using modulo 2 so that half the kilobots
  // the edge in one direction and the other half in the
  // other direction. This NEEDS to change, very sloppy code
  if(kilo_uid % 2 == 1){
  if(find_nearest_N_dist() > desired_dist)
  {
    if(get_move_type() == LEFT)
      spinup_motors();
    set_motors(0, kilo_turn_right);
    set_move_type(RIGHT);
  }
  else
  {
    if(get_move_type() == RIGHT)
      spinup_motors();
    set_motors(kilo_turn_left, 0);
    set_move_type(LEFT);
    }
  }
  else{
    if(find_nearest_N_dist() > desired_dist)
    {
      if(get_move_type() == RIGHT)
         spinup_motors();
      set_motors(kilo_turn_left, 0);
      set_move_type(LEFT);
    }
    else
    {
      if(get_move_type() == LEFT)
        spinup_motors();
      set_motors(0, kilo_turn_right);
      set_move_type(RIGHT);
    }
  }
}

void loop()
{
  //receive messages
  receive_inputs();
  set_gradient();
  // within stopping threshold
  if (mydata->my_reading < 70){
    spinup_motors();
    set_motors(0, 0);
    set_move_type(STOP);
    set_bot_state(LISTEN);
    set_color(RGB(0,0,1));
    mydata->own_gradient = 0;
  }
  // kilobot is NOT an adhesion site
  else if (mydata->own_gradient != 0){
    bool highest = true;
    bool only_mover = true;
    mydata->lock = false;
    // check to see if any neighbors are moving and if have
    // highest gradient
    for (int i = 0; i < mydata->N_Neighbors; i++){
      if (mydata->neighbors[i].n_bot_state == MOVE){
        only_mover = false;
      }
      if (mydata->neighbors[i].grad > mydata->own_gradient){
        highest = false;
      }
    }
    // Follow the edge if both no neighbors are moving and have
    // highest gradient
    if(highest && only_mover)
    {
      set_color(RGB(1,0,0));
      follow_edge();
    }
    // Stay stationary
    else{
      set_color(RGB(0,1,0));
      set_bot_state(LISTEN);
      spinup_motors();
      set_motors(0, 0);
      set_move_type(STOP);
    }
  }
  // If the kilobot is all alone
  else if (mydata->N_Neighbors == 0){
    // it appears that 14seconds is around how long it
    // takes for the kilobot to turn 180 degrees
    if (!mydata->lost){
        mydata->momentum_check = kilo_ticks;
    }
    mydata->lost = true;
    if (kilo_ticks - mydata->momentum_check > 515){
      spinup_motors();
      set_motors(kilo_turn_left, kilo_turn_right);
      set_move_type(STRAIGHT);
    }
    else
      set_color(RGB(1,1,1));
  }
  // Otherwise, the kilobot is an adhesion site
  else {
    // If far enough away from other kilobots stop moving
    if (find_nearest_N_dist() > 55){
      spinup_motors();
      set_motors(0, 0);
      set_move_type(STOP);
      mydata->lock = true;
    }
    // Otherwise move forward
    else if (!mydata->lock && mydata->N_Neighbors > 0){
      spinup_motors();
      set_motors(kilo_turn_left, kilo_turn_right);
      set_move_type(STRAIGHT);
    }
    set_color(RGB(0,0,1));
    set_bot_state(LISTEN);
  }
  setup_message(); // prepare the next message
}
/*
 * Beyond this point is all helper code for the simulator
 */
extern char* (*callback_botinfo) (void);
char *botinfo(void);
int16_t callback_lighting(double, double);
int main(void)
{
  kilo_init();
#ifdef DEBUG
  // setup debugging, i.e. printf to serial port, in real Kilobot
  debug_init();
#endif
  SET_CALLBACK(botinfo, botinfo);
  SET_CALLBACK(reset, setup);
  SET_CALLBACK(lighting, callback_lighting);
  RB_init();                       // initialize ring buffer
  kilo_message_rx = rxbuffer_push;
  kilo_message_tx = message_tx;    // register our transmission function
  kilo_start(setup, loop);
  return 0;
}
#ifdef SIMULATOR
// provide a text string for the status bar, about this bot
static char botinfo_buffer[10000];
//print out information about the bot
char *botinfo(void)
{
  int n;
  char *p = botinfo_buffer;
  n = sprintf (p, "ID: %d ", kilo_uid);
  p += n;
  n = sprintf (p, "my reading: %i ", mydata->my_reading);
  p += n;
  n = sprintf (p, "light: %d ", get_ambientlight());
  p += n;
  n = sprintf (p, "transmitted reading: %i ", (mydata->transmit_msg.data[4] << 8) | mydata->transmit_msg.data[5]);
  p += n;
  n = sprintf (p, " transmitted ttl: %i ", (mydata->transmit_msg.data[7]));
  p += n;
  n = sprintf (p, "gradient: %i ", mydata->own_gradient);
  p += n;

  return botinfo_buffer;
}

/*
 * Uses euclidean distance to setup a lighting callback function
 * Lower values returned values correspond to higher light
 * Light readings degrade linearly
 */
int16_t callback_lighting(double x, double y){
  double light_y = 0;
  double light_x = -150;
  double dist_x = pow(light_x + x, 2);
  double dist_y = pow(light_y + y, 2);
  double dist_c = sqrt(dist_x + dist_y);
  return (int16_t)dist_c;
}
#endif

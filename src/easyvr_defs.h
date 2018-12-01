//
// Created by swood on 12/1/18.
//

#ifndef I2SDAC_EASYVR_DEFS_H
#define I2SDAC_EASYVR_DEFS_H


//Groups and Commands
enum Groups
{
    GROUP_1  = 1,
};

enum Group1
{
    G1_JASON_CRANE = 0,
    G1_RISE_HEADLESS = 1,
    G1_AND_RIDE = 2,
    G1_SURRENDER = 3,
    G1_AGATHE_VAN_BRUNDT = 4,
    G1_HADEWYCH = 5,
    G1_BROM_BONES = 6,
    G1_JOEY_OSARIO = 7,
    G1_VALERIE_MALL = 8,
    G1_JESSICA_BRIDGE = 9,
    G1_EDDIE_MARTINEZ = 10,
    G1_GORYBROOK_ROAD = 11,
    G1_KNOCK_KNOCK = 12,
    G1_RISE_HEADLESS_AND_RIDE = 13,
    G1_HEDWIG_VAN_BRUNDT = 14,
};

//Grammars and Words
enum Wordsets
{
    SET_1  = -1,
    SET_2  = -2,
    SET_3  = -3,
};

enum Wordset1
{
    S1_ACTION = 0,
    S1_MOVE = 1,
    S1_TURN = 2,
    S1_RUN = 3,
    S1_LOOK = 4,
    S1_ATTACK = 5,
    S1_STOP = 6,
    S1_HELLO = 7,
};

enum Wordset2
{
    S2_LEFT = 0,
    S2_RIGHT = 1,
    S2_UP = 2,
    S2_DOWN = 3,
    S2_FORWARD = 4,
    S2_BACKWARD = 5,
};

enum Wordset3
{
    S3_ZERO = 0,
    S3_ONE = 1,
    S3_TWO = 2,
    S3_THREE = 3,
    S3_FOUR = 4,
    S3_FIVE = 5,
    S3_SIX = 6,
    S3_SEVEN = 7,
    S3_EIGHT = 8,
    S3_NINE = 9,
    S3_TEN = 10,
};

#endif //I2SDAC_EASYVR_DEFS_H

#include "SquarePositions.h"

double  squarePos[8][8][3]  = {
  {{126.0,144.0,-255.0},{126.0,108.0,-255.0},{126.0,72.0,-255.0},{126.0,36.0,-255.0},{126.0,0.0,-255.0},{126.0,-36.0,-255.0},{126.0,-72.0,-255.0},{126.0,-108.0,-255.0}},
  {{90.0,144.0,-255.0},{90.0,108.0,-255.0},{90.0,72.0,-255.0},{90.0,36.0,-255.0},{90.0,0.0,-255.0},{90.0,-36.0,-255.0},{90.0,-72.0,-255.0},{90.0,-108.0,-255.0}},
  {{54.0,144.0,-255.0},{54.0,108.0,-255.0},{54.0,72.0,-255.0},{54.0,36.0,-255.0},{54.0,0.0,-255.0},{54.0,-36.0,-255.0},{54.0,-72.0,-255.0},{54.0,-108.0,-255.0}},
  {{18.0,144.0,-255.0},{18.0,108.0,-255.0},{18.0,72.0,-255.0},{18.0,36.0,-255.0},{18.0,0.0,-255.0},{18.0,-36.0,-255.0},{18.0,-72.0,-255.0},{18.0,-108.0,-255.0}},
  {{-18.0,144.0,-255.0},{-18.0,108.0,-255.0},{-18.0,72.0,-255.0},{-18.0,36.0,-255.0},{-18.0,0.0,-255.0},{-18.0,-36.0,-255.0},{-18.0,-72.0,-255.0},{-18.0,-108.0,-255.0}},
  {{-54.0,144.0,-255.0},{-54.0,108.0,-255.0},{-54.0,72.0,-255.0},{-54.0,36.0,-255.0},{-54.0,0.0,-255.0},{-54.0,-36.0,-255.0},{-54.0,-72.0,-255.0},{-54.0,-108.0,-255.0}},
  {{-90.0,144.0,-255.0},{-90.0,108.0,-255.0},{-90.0,72.0,-255.0},{-90.0,36.0,-255.0},{-90.0,0.0,-255.0},{-90.0,-36.0,-255.0},{-90.0,-72.0,-255.0},{-90.0,-108.0,-255.0}},
  {{-126.0,144.0,-255.0},{-126.0,108.0,-255.0},{-126.0,72.0,-255.0},{-126.0,36.0,-255.0},{-126.0,0.0,-255.0},{-126.0,-36.0,-255.0},{-126.0,-72.0,-255.0},{-126.0,-108.0,-255.0}}
};


bool getSquarePos(char hor, char vert, double& x, double& y, double& z)
{
  int horIdx  = tolower(hor) - 'a';
  int vertIdx = vert - '1';

  if ((horIdx < 0) || (horIdx > 7) || (vertIdx < 0) || (vertIdx > 7)) {
    return false;
  }

  x = squarePos[horIdx][vertIdx][0];
  y = squarePos[horIdx][vertIdx][1];
  z = squarePos[horIdx][vertIdx][2];

  return true;
}

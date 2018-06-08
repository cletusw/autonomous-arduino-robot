// From https://github.com/ninetreesdesign/median/blob/72bf737f95263b3f7b09d2aa6cc240b37f493bee/ds_median_filter_demo.ino
void sort(int *j,  int *k) {
  int x = *j;
  int y = *k;
  if (*j > *k) {
    *k = x;
    *j = y;
  }
}

int median5(int readings[5]) {
  // Network for N=5, using Bose-Nelson Algorithm
  sort(&readings[0], &readings[1]);
  sort(&readings[3], &readings[4]);
  sort(&readings[2], &readings[4]);
  sort(&readings[2], &readings[3]);
  sort(&readings[0], &readings[3]);
  sort(&readings[0], &readings[2]);
  sort(&readings[1], &readings[4]);
  sort(&readings[1], &readings[3]);
  sort(&readings[1], &readings[2]);

  return readings[2]; // median value
}


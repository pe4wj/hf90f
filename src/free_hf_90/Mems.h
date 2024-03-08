typedef struct { // frequency memory
  int freq_; // channel frequency
  char* mode_; // name of the channel
  } Freqs;



const Freqs freqs[] = {
  {3630000, "LSB"},
  {3692000, "LSB"},
  {3791000, "LSB"},
  {7073000, "LSB"},
  {14292000, "USB"},
  {14285000, "USB"},
  {18150000, "USB"},
  {21250000, "USB"},
  {28500000, "USB"},
  {8957000, "USB"},
  {3413000, "USB"},
  {5505000, "USB"}
};

uint8_t N_MEMS = 12;

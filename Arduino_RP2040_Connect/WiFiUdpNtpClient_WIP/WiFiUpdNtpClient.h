struct CurrentTime {
  unsigned long utc_unix_time;
  unsigned long local_unix_time;
  uint8_t hours, minutes, seconds;
  String time_stamp;
};
# Summary
This project investigated the reliability of the Universal Asynchronous Receiver-
Transmitter (UART) communication between the Nicla Vision camera and the Norwegian
University of Science and Technology (NTNU) Simultaneous Localization
and Mapping (SLAM) robot. A test framework was developed to monitor communication
performance by tracking received packets, timeouts, UART errors, and
missing or suspicious sequence numbers. A total of 40 tests were performed to
investigate the effects of baud rate, camera processing, servo activity, and cable
configuration. The results showed that camera image processing had the greatest
impact on communication reliability, as all tests with image processing off achieved
a 0% failure rate. With image processing on, 57600 baud provided the lowest failure
rate. Servo activity had no significant effect, while changing the cable configuration
generally reduced reliability. Several potential improvements were identified, including
adjusting the baud rate and timeout or moving image processing to the robot
platform. However, no final improvements were implemented. The results will provide
a basis for further development and testing of a more reliable communication
system.

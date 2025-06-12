# Team #3195 COSMOS

![banner](teambanner.png)

Cansat Competition Description:
- The Cansat competition is a design-build-fly competition that provides teams with an
opportunity to experience the design life-cycle of an aerospace system. The Cansat
competition is designed to reflect a typical aerospace program on a small scale and includes
all aspects of an aerospace program from the preliminary design review to post flight review.
The mission and its requirements are designed to reflect various aspects of real world
missions including telemetry, communications, and autonomous operations. Each team is
scored throughout the competition on real-world deliverables such as schedules, design
review presentations, and demonstration flights.

Mission Overview:
- Design a Cansat that consists of a payload and a container that mounts on top of the rocket.
The payload rests inside the container at launch and includes the nose cone as part of the
payload.
- The container with the payload shall deploy from the rocket when the rocket reaches peak
altitude and the rocket motor ejection forces a separation. The container with the payload
shall descend at a rate of no more than 20 meters/second using a parachute that
automatically deploys at separation.
- At 75% peak altitude, the payload shall separate from the container and descend using an
auto-gyro descent control system until landing. The descent rate shall be 5 meters/second.
- A video camera shall show the separation of the payload from the container and the auto-gyro
functioning. A second video camera shall be pointing downward at 45 degrees from nadir
and oriented north during descent and be spin stabilized so that the view of the earth is not
rotating.
- The Cansat shall collect sensor data during ascent and descent and transmit the data to a
ground station at a 1 Hz rate. The sensor data shall include interior temperature, battery
voltage, altitude, auto-gyro rotation rate, acceleration, rate, magnetic field, and GPS position.

Post-Flight Cansat Payload:

![Cansat](cansat.png)

Ground Station during Landing:

![Ground Station](groundstation.png)

Software Contributors:
- `GCS`:
  - Diego Romero-Cardona

- `FSW`:
  - Caleb Wiley
  - Akhil Samiraju
  - Barrett Twining
  - Diego Romero-Cardona

- `PID Board`:
  - Caleb Wiley
  - Barrett Twining
  - Luke Wiggins

Competition Flight Data:
|TEAM_ID|MISSION_TIME|PACKET_COUNT|MODE|STATE        |ALTITUDE|TEMPERATURE|PRESSURE|VOLTAGE|GYRO_R |GYRO_P |GYRO_Y  |ACCEL_R|ACCEL_P|ACCEL_Y|MAG_R|MAG_P|MAG_Y|AUTO_GYRO_ROTATION_RATE|GPS_TIME|GPS_ALTITUDE|GPS_LATITUDE|GPS_LONGITUDE|GPS_SATS|CMD_ECHO   |TEAM_NAME|
|-------|------------|------------|----|-------------|--------|-----------|--------|-------|-------|-------|--------|-------|-------|-------|-----|-----|-----|-----------------------|--------|------------|------------|-------------|--------|-----------|---------|
|3195   |18:26:32    |972         |F   |LAUNCH_PAD   |-0.9    |40.6       |915.5   |7.9    |-0.79  |-1.28  |0.24    |-0.02  |0.00   |-0.49  |-42.6|-3.0 |61.6 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:33    |973         |F   |LAUNCH_PAD   |-0.9    |40.7       |915.5   |7.9    |-0.67  |-0.98  |0.37    |-0.02  |-0.00  |-0.49  |-43.5|-2.9 |59.0 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:38    |978         |F   |ASCENT       |186.3   |40.7       |895.4   |7.9    |-10.68 |7.63   |-123.54 |-0.10  |-0.05  |-4.00  |-45.7|-7.9 |61.9 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:39    |979         |F   |ASCENT       |455.9   |40.6       |867.0   |7.9    |24.29  |-9.09  |-300.78 |-0.12  |0.09   |0.64   |-37.9|-2.8 |63.8 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:40    |980         |F   |ASCENT       |478.8   |40.6       |864.7   |7.9    |3.36   |-19.84 |-251.46 |-0.04  |0.14   |0.51   |-33.1|-3.6 |62.0 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:41    |981         |F   |APOGEE       |500.4   |40.6       |862.4   |7.9    |-1.53  |-13.43 |-183.53 |0.01   |0.15   |0.31   |-6.4 |-20.7|62.3 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:42    |982         |F   |APOGEE       |530.2   |40.5       |859.3   |7.9    |8.67   |-17.94 |-138.37 |-0.02  |0.10   |0.20   |-46.8|-33.5|58.3 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:43    |983         |F   |APOGEE       |557.5   |40.5       |856.5   |7.9    |5.19   |-14.28 |-94.97  |-0.02  |0.06   |0.12   |-5.4 |-31.5|58.2 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:44    |984         |F   |APOGEE       |579.7   |40.5       |854.3   |7.9    |4.88   |-12.02 |-74.28  |-0.00  |0.03   |0.07   |-32.1|-4.0 |63.1 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:45    |985         |F   |APOGEE       |597.4   |40.5       |852.4   |7.9    |1.59   |-8.97  |-56.64  |-0.00  |0.01   |0.03   |-49.3|-27.0|60.7 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:46    |986         |F   |APOGEE       |611.3   |40.6       |851.0   |7.9    |-27.71 |-4.70  |-52.43  |-0.00  |-0.03  |0.05   |-36.9|-36.6|62.3 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:47    |987         |F   |APOGEE       |616.2   |40.6       |850.5   |7.9    |23.19  |31.19  |-55.60  |0.01   |-0.02  |0.02   |-15.8|-34.8|61.7 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:48    |988         |F   |DESCENT      |612.7   |40.6       |850.9   |7.9    |-62.93 |243.71 |-11.78  |0.20   |0.02   |-0.08  |9.2  |-9.7 |-12.0|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:49    |989         |F   |DESCENT      |603.2   |40.6       |851.9   |7.9    |-166.63|83.56  |115.36  |0.48   |-0.20  |0.09   |3.3  |-28.4|-22.3|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:51    |991         |F   |DESCENT      |569.9   |40.6       |855.3   |7.9    |355.41 |88.93  |179.75  |0.45   |0.10   |0.11   |-25.0|-19.3|-34.5|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:52    |992         |F   |DESCENT      |551.7   |40.6       |857.1   |7.9    |20.14  |78.74  |325.38  |-0.07  |0.31   |0.35   |-11.4|9.3  |-19.6|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:53    |993         |F   |DESCENT      |533.0   |40.6       |859.1   |7.9    |156.19 |229.92 |301.57  |-0.15  |0.42   |0.35   |-12.9|-13.2|-32.8|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:54    |994         |F   |DESCENT      |510.8   |40.6       |861.4   |7.9    |146.97 |25.39  |286.56  |0.38   |-0.23  |0.42   |2.1  |-38.5|-21.2|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:55    |995         |F   |DESCENT      |490.6   |40.6       |863.4   |7.9    |-86.98 |-158.63|330.69  |0.14   |0.61   |0.31   |-38.7|3.5  |-26.8|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:56    |996         |F   |DESCENT      |472.7   |40.6       |865.3   |7.8    |69.27  |195.80 |316.77  |0.37   |0.50   |0.32   |-36.7|3.0  |-26.7|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:57    |997         |F   |DESCENT      |451.3   |40.6       |867.5   |7.9    |134.70 |77.64  |407.17  |0.46   |0.32   |0.41   |6.7  |-10.9|-18.6|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:58    |998         |F   |DESCENT      |434.0   |40.6       |869.3   |7.9    |-62.81 |136.17 |393.92  |-0.05  |0.22   |0.41   |-57.7|-2.6 |-18.7|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:26:59    |999         |F   |DESCENT      |414.3   |40.6       |871.4   |7.9    |-250.37|206.73 |394.04  |-0.22  |0.51   |0.24   |-26.8|12.5 |-19.1|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:00    |1000        |F   |DESCENT      |396.6   |40.6       |873.2   |7.9    |-7.32  |90.88  |316.10  |-0.02  |0.25   |0.43   |-58.1|1.8  |-17.3|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:01    |1001        |F   |DESCENT      |379.0   |40.6       |875.1   |7.9    |-269.53|371.89 |410.64  |-0.24  |0.67   |-0.02  |-29.9|13.9 |-22.2|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:02    |1002        |F   |DESCENT      |358.8   |40.6       |877.2   |7.8    |-180.79|100.16 |443.54  |-0.40  |0.28   |0.37   |-53.6|6.0  |-18.8|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:03    |1003        |F   |PROBE_RELEASE|329.8   |40.4       |880.2   |7.9    |156.86 |-34.48 |512.76  |-0.11  |-0.04  |0.39   |-47.8|-4.9 |-23.5|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:04    |1004        |F   |PROBE_RELEASE|306.5   |39.8       |882.7   |7.8    |171.81 |-233.03|429.14  |0.25   |-0.46  |0.24   |-32.3|-46.6|-22.5|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:05    |1005        |F   |PROBE_RELEASE|290.9   |39.4       |884.3   |7.9    |195.19 |-323.36|325.81  |0.26   |-0.57  |0.16   |-14.8|-63.6|-2.6 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:06    |1006        |F   |PROBE_RELEASE|275.7   |39.1       |885.9   |7.9    |205.02 |-142.40|310.79  |-0.13  |-0.16  |0.31   |-18.2|-9.4 |-30.4|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:07    |1007        |F   |PROBE_RELEASE|256.2   |38.6       |888.0   |7.9    |125.61 |-119.32|473.08  |0.42   |-0.12  |0.36   |-9.1 |-58.5|-6.7 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:08    |1008        |F   |PROBE_RELEASE|238.0   |38.2       |889.9   |7.9    |191.59 |-214.36|288.39  |-0.32  |0.07   |0.37   |-22.8|-3.7 |-29.0|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:09    |1009        |F   |PROBE_RELEASE|219.2   |37.8       |891.9   |7.9    |147.58 |15.56  |386.84  |-0.06  |0.00   |0.43   |-22.3|-4.4 |-31.7|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:10    |1010        |F   |PROBE_RELEASE|200.3   |37.4       |893.9   |8.0    |131.29 |-99.98 |549.99  |-0.41  |-0.09  |0.40   |-50.2|5.7  |-20.6|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:11    |1011        |F   |PROBE_RELEASE|180.6   |37.2       |896.0   |8.0    |18.13  |-7.69  |472.23  |0.04   |0.21   |0.47   |-10.3|-10.8|-28.9|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:12    |1012        |F   |PROBE_RELEASE|160.5   |36.9       |898.2   |7.9    |-168.70|52.31  |724.73  |0.42   |0.11   |0.42   |-23.5|-25.8|-30.3|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:13    |1013        |F   |PROBE_RELEASE|147.7   |35.9       |899.5   |8.0    |-150.45|-159.18|79.22   |0.09   |-0.11  |0.46   |-1.4 |-33.5|-23.6|0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:14    |1014        |F   |PROBE_RELEASE|138.8   |34.6       |900.5   |7.9    |137.51 |122.25 |-408.87 |-0.18  |0.10   |-0.44  |-75.2|-29.1|21.8 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:15    |1015        |F   |PROBE_RELEASE|113.2   |33.8       |903.2   |7.9    |64.15  |-101.81|-1292.79|-0.10  |0.34   |-0.60  |-76.4|-3.2 |23.0 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:16    |1016        |F   |PROBE_RELEASE|99.8    |33.1       |904.7   |7.9    |-15.20 |245.54 |-1796.75|0.48   |-0.07  |-0.98  |-22.2|-44.7|60.2 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:17    |1017        |F   |PROBE_RELEASE|83.2    |32.7       |906.5   |7.9    |59.20  |8.79   |-1999.76|0.42   |0.18   |-0.61  |-7.2 |-42.0|57.1 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:18    |1018        |F   |PROBE_RELEASE|72.7    |32.6       |907.6   |7.9    |28.56  |-3.36  |-1999.76|-0.06  |-0.16  |-0.48  |-5.2 |-23.6|60.7 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:19    |1019        |F   |PROBE_RELEASE|63.5    |32.4       |908.6   |7.9    |33.87  |91.13  |-1999.76|-0.17  |0.33   |-0.62  |-6.9 |-12.3|61.6 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:20    |1020        |F   |PROBE_RELEASE|54.1    |32.3       |909.6   |7.9    |79.65  |23.56  |-1999.76|0.05   |0.16   |-0.48  |4.5  |-7.5 |53.2 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:21    |1021        |F   |PROBE_RELEASE|45.0    |32.1       |910.6   |7.8    |30.40  |82.82  |-1999.76|0.24   |0.05   |-0.56  |-23.2|-16.7|68.2 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:22    |1022        |F   |PROBE_RELEASE|35.6    |32.0       |911.6   |7.9    |-2.08  |62.38  |-1999.76|0.10   |0.08   |-0.49  |-63.0|-12.4|55.5 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:23    |1023        |F   |PROBE_RELEASE|26.1    |31.9       |912.6   |7.8    |40.34  |34.42  |-1999.76|0.10   |0.09   |-0.43  |-22.2|-6.5 |66.8 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:24    |1024        |F   |PROBE_RELEASE|16.4    |31.8       |913.7   |7.9    |15.26  |23.68  |-1999.76|0.26   |0.17   |-0.56  |-54.9|-31.0|57.9 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:25    |1025        |F   |PROBE_RELEASE|8.5     |31.3       |914.5   |7.8    |16.97  |-82.09 |181.58  |-0.06  |-0.02  |-0.12  |-76.6|-21.8|29.5 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |ST_UTC_TIME|COSMOS   |
|3195   |18:27:26    |1026        |F   |PROBE_RELEASE|8.3     |31.5       |914.6   |7.9    |3.85   |2.87   |-1.89   |-0.33  |-0.01  |-0.37  |-74.8|-20.1|41.1 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |PARTY_ON   |COSMOS   |
|3195   |18:27:28    |1028        |F   |LANDED       |8.1     |31.6       |914.6   |7.9    |-0.67  |0.18   |0.79    |-0.32  |-0.02  |-0.37  |-74.7|-20.5|40.0 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |PARTY_ON   |COSMOS   |
|3195   |18:27:29    |1029        |F   |LANDED       |8.0     |31.7       |914.6   |7.9    |1.04   |-0.79  |-0.67   |-0.33  |-0.01  |-0.37  |-73.7|-21.4|37.4 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |PARTY_ON   |COSMOS   |
|3195   |18:27:30    |1030        |F   |LANDED       |7.9     |31.8       |914.6   |7.9    |-0.37  |1.65   |0.00    |-0.33  |-0.01  |-0.37  |-72.8|-20.6|41.2 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |PARTY_ON   |COSMOS   |
|3195   |18:27:31    |1031        |F   |LANDED       |7.8     |31.9       |914.6   |7.9    |0.12   |1.46   |0.79    |-0.32  |-0.01  |-0.37  |-73.8|-20.1|41.4 |0                      |00:00:00|0.0         |0.00000     |0.00000      |0       |PARTY_ON   |COSMOS   |

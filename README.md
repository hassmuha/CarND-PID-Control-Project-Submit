# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
[//]: # (Image References)

[clip1]: https://youtu.be/R-c7kgNEGNQ "P_Result"
[clip2]: https://youtu.be/MI_S1LyLZzI "PD_Result"
[clip3]: https://youtu.be/8yHsKbEw-Ks "PID_Result"
[clip4]: https://youtu.be/e5RhgDL2300 "Complete_Lap"


---

* Source Code can be found [here](https://github.com/hassmuha/CarND-PID-Control-Project-Submit/tree/master/src)
* Compilation Result ./particle_filter can be found [here](https://github.com/hassmuha/CarND-PID-Control-Project-Submit/tree/master/build)
* Video Clipping for the simulator results can be found here [here](https://github.com/hassmuha/CarND-PID-Control-Project-Submit/tree/master/Result)

---

### Key Points
The PID Controller project has been developed in the same way as it was taught in the lecture
* Started with P Controller first, then introducing the Differential part and at the end Integral part
* Twiddle has been used to select the optimum parameters for P, I and D parts
* Twiddle algorithm is run with different update parameter ranging from 50 till 500, pid.CoefUpdate (main.cpp:line38). At the end optimum initial parameters values has been selected which were Kp = .15, Ki = .0002, Kd=2 and with update parameter of 200
* In order that for every evaluation with change of PID parameters, first half of iterations within update duration didn't contribute towards calculating the mean squared CTE (PID.cpp:line57). This allows us to converge the algorithm first based on new coefficients and then calculate the mean squared CTE.
* For the first 500 the PID controller run only on initial selected parameters without any parameters update using Twiddle. This allows the car to throttle to its maximum selected speed and reach the steady state. (main.cpp:line79)
* Final Result Video [here][clip4]
* Moreover to see the effect of PID separarely following clips were made:
1) P Result only with Kp = .15 [here][clip1]. We can see the oscillation even on straight road
2) PD Result only with Kp = .15, Kd=2 [here][clip2]. We can see the oscillation problem has been improved
3) PID Result only with Kp = .15, Kd=2, Ki = .0002 [here][clip3]. Seems like the integral part didn't contribute much in further improving the result, indication of small value of system biasness

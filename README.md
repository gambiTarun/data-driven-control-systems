**A REPORT ON**

**DESIGN OF DATA-DRIVEN            CONTROLLER AND ITS APPLICATION**

**BY**

**TARUNBIR SINGH GAMBHIR**

**B.E (Hons.)  Electrical and Electronics Engineering            BIRLA INSTITUTE OF TECHNOLOGY & SCIENCE, PILANI**

**AT** 

**THE DEPARTMENT OF              CONTROL SYSTEMS ENGINEERING** 

![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.001.png)

**HIROSHIMA UNIVERITY JULY 2019** 

Acknowledgements![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.002.png)

I would like to thank **Prof. Toru Yamamoto** for giving me chance to work on this research topic and gain experience employing this control scheme on a tank system. 

I would also like to thank **Dr. T.Kinoshita** for all his time and guidance for the successful completion and implementation of the project. 

I would like to thank **Keishi Fujiwara** for all his time and effort in helping me understand basic concepts and clearing away my doubts. 

Abstract 

Most of the real-world systems exhibit nonlinearity and are therefore governed by nonlinear differential equations. Control design schemes that can deal with such  systems  efficiently  are  designed  and  implemented  in  this  project.  A conventional I-PD controller is modified to a data-driven controller such that the PID parameters become dynamic in nature and can adapt to get the desired system  output.  The  idea  is  to  use  data  collected  from  old  experiments  to generate the PID parameters that can adapt to the nonlinearity in the output. The change in PID parameters can be realised in an online manner, where the size of the database is dynamic, or an offline manner, where the database is fixed in size.   

3
Table of Contents

Acknowledgements  2 Abstract  3 Introduction  5 Design of DD PID Control  6 

Simulation Example 10 Design of DD FRIT Control  12 Simulation Example  14 Experiment on Tank System  16 References  19 

4

Introduction 

The real systems have nonlinearity that can be dealt to some extent using a PID control. But systems with high nonlinearly cannot be controlled by a PID controller with fixed parameters. Dynamic parameters allow for better control of systems with strong nonlinearity. In this project, a technique for self-tuning PID control is designed and implemented using recorded data of a system. However, for the controller to work on a nonlinear system the data has be recorded from a nonlinear system. 

In the past, many complex control algorithms using the aid of neural networks or genetic algorithms have been proposed for systems with strong nonlinearity. They do come  with  their  drawbacks  including  high  computational  cost  and  large  data requirement.   

An online data-driven control scheme is designed that reduces the data requirements and thereby the computational costs. The PID parameters are adequately adapted in proportion  to  the  control  errors,  and  the  updated  parameters  are  stored  in  the database. The database is also stripped of any redundant data points in real-time to reduce the computational and memory storage requirements. 

Furthermore, an offline data-driven based fictious reference iterative tuning (DD FRIT) control scheme is designed and implemented. In this algorithm, a reference signal is generated from the initial closed-loop data and control parameters, and the optimal PID parameters are calculated using this signal. The major difference from the online method is that new data is not added to the database in real-time. 

5

Design of DD PID Control (Online) 

The system is considered a discrete-time nonlinear system: 

( )  =  ( ( − 1))*  (1) ( − 1)  ≔ [ ( − 1),…, ( − ), ( − 1),…, ( − )]   (2) 

Where y(t) denotes system output and f(.) denotes the nonlinear function. Here,  (t) denotes the ‘information vector’.  

A conventional digital PID controller is given by: 

- ( ) = ∆ ( ) +  ( ) +  ∆2 ( )*  (3) 

Where K denotes the PID parameters, Δ is the difference operator and e(k) is the control error in the discrete-time system.  

Using the fact that Δ2e(k) = e(k) – 2e(k-1) + e(k-2), and assuming that the reference values are constant we can write the above equation as follows: 

- ( ) = ( ) −  ∆ ( ) −  ∆2 ( )*  (4) 

This PID control law is known as the I-PD control law. We shall be using the I-PD control law in the designing of the data driven control scheme.  

For systems with strong nonlinearities, it is hard to obtain good control with fixed PID parameters. Therefore, time variant PID parameters are introduced in the following control law: 

- ( ) = ( ) ( ) −  ( )∆ ( ) −  ( )∆2 ( )*  (5) 

u(t) can be rewritten as: 

( ) = ( ′( ))*  (6) ′( )  ≔ [ ( ), ( ), ( ), ( − 1), ( − 2), ( − 1)]*  (7) 

( ) ≔ [ ( ), ( ), ( )]*  (8) 

Where g(.) denotes the function from (5). By substituting (6) and (7) into (1) and (2) the following can be derived: 

7

( ) =  ( ( )) 

( ) ∶= [ ( + 1), ( ),…, ( − + 1), 

( ), ( − 1),…, ( −

(9) (10) 

+ 1)]* 



It is not possible to obtain future output y(t+1) at time t, so y(t+1) is replaced with r(t+1) as the value of y(t+1) —> r(t+1) using a controller. Therefore,  

( ) ∶= [ ( + 1), ( ),…, ( − + 1),  (11) 

( ), ( − 1),…, ( − + 1)]* 

The initial database is created from instances of the information vector (9) generated by the initial run with fixed PID parameters.  

( ) = [ ( ), ( )]   *,    j=1,2,…,N(0)*  (12) 

This database is used for PID parameter tuning in real-time. At every instance, k- nearest-neighbours of the query information vector are computed from the database. For this purpose, distance of the query vector to all the instances in the database is calculated using the ℒ -1 norm: 

+ +1

( ) − ( )

( ( ), ′( )) =  ∑ | | ,

( ) −  ( ) (13) =1

`  `j = 1,2,…,N(t) 

N(t) denotes the number of information vectors stored at that instance. Suitable PID gains are computed from the parameters  of k-nearest-neighbour of  the query as following: 

( ) = ∑ ( )   ,          ∑ = 1 (14) 

=0 =1

After calculating the parameters from the database, they are updated using steepest gradient descent method for reducing the cost function J. 

( ) =  ( ) −  ( + 1) (15) ( )

≔ [ , , ] (16) 

Where  gives the learning rate of the gradient descent algorithm and J(t) is the cost function defined by: 

( ) = 1 ( )2 (17) 

2

( ) =  ( ) − ( )  (18) 

(t) denotes the output of the reference model and is defined by: 

−1 (1) (19) ( ) =  ( )

( −1)

( −1) = 1 + −1 + −2 (20) 

1 2

Here,  ( −1) is designed based on the desired rise-time and the damping property. The gradients in (15) can be derived as following: 

( + 1) ( + 1) (21) 

- ( + 1)( ( ) − ( − 1))

( ) ( )

( + 1) ( + 1) (22) 

- − ( + 1)( ( ) − ( ))

( ) ( )

( + 1) ( + 1) (23) 

- ( + 1)( ( ) − 2 ( − 1) + ( − 2))

( ) ( )

The  next  step  is  to  check  for  data  redundancy  and  remove  the  instances  in  the database that are similar. This can be done by setting a threshold distance below which the instances in the database are considered redundant. In this case two conditions are used to filter out the redundant data in the database. First condition, 

`  `( ( ), ′( )) ≤  ,        = 1,2,…, ( ) − .  (24) 

1

extracts the instances in the databases that satisfy the above condition. Furthermore, these information vectors are passed to the Second condition:   

3

( ) − ( ) 2 (25) 

∑ { ( ) } ≤ 2

=1

The vectors extracted are deleted from the database as they have high similarity with the query information vector. The parameters  and  are set by trial and error. 

1 2

The query information vector along with the calculated  ( ) is added into the database and the process is repeated at every instance of time. 

9
Simulation Example 

We will use a Hammerstein model to depict the usefulness of the data-driven control method. The following two systems are switched between during the runtime. 

System 1 (0<t<70) 

`  `( ) = 0.6 ( − 1) − 0.1 ( − 2) + 1.2 ( − 1) − 0.1 ( − 2)  (26)   ( ) = 1.5 ( ) − 1.5 2( ) − 0.5 3( )  (27) 

System 2 (70≤t<200) 

`  `( ) = 0.6 ( − 1) − 0.1 ( − 2) + 1.2 ( − 1) − 0.1 ( − 2)  (28)   ( ) = 1.5 ( ) − 1.5 2( ) − 1.0 3( )  (29) 

The reference signal is given as: 

0.5(0 ≤ < 50)

( ) = { 1.0(50 ≤ < 100) (30) 

2.0(100 ≤ < 150)

1.5(150 ≤ ≤ 200)

The database is updated for 10 epochs. The following hyper-parameters were set for this example. 



|Orders of the information vector |Number of Neighbours |Learning Rates |Coefficients to remove redundant data |Initial Number of Data |
| :-: | - | - | :-: | :-: |
|<p>- 3 </p><p>- 2 </p>|= 6 |<p>- 0.8  </p><p>- 0.8 </p><p>- 0.2 </p>|<p>- 0.0005 </p><p>- 0.0001 </p><p>1 2</p>|(0) = 6 |
|Fixed PID parameters |<p>K = 0.486, K = 0.227, K = 0.122 </p><p>P I D</p>|
|Characteristic Polynomial Coefficients |<p>t = -0.271, t = 0.0183  </p><p>1 2</p>|

11
` `The simulation results are below: 

![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.003.jpeg) ![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.004.jpeg)

*Figure 1 Fixed PID output*  *Figure 2 Data Driven PID output* 

![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.005.jpeg) ![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.006.jpeg)

*Figure 3 Trajectory of PID parameters*  *Figure 4 Comparison of both the outputs* 

![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.007.jpeg) ![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.008.jpeg)

*Figure 5 Vectors removed from the database*  *Figure 6 Cost function over the epochs* 

13

Design of DD FRIT Control (Offline) 

The database is created in a similar way from instances of the information vector generated by the run with fixed PID parameters. The database in this method is used to find the optimal control parameter vector.  

( ) = [ ( ), ( )]   *,    j=1,2,…,N*  (31) 

0 0

Where N is the size of the database. The database is size not increased with every query as it was the case with DD PID control scheme.  

A fictious reference signal is generated from the closed loop data and the control parameters in the database. The value of this reference signal can be derived as: 

r(t) = y (t) +  1 {Δu (t) + K (t)Δy (t) + K (t)Δ2y (t)}*  (32) 

0 K (t) 0 P 0 D 0

I

After getting the reference signal, the output reference signal  ( ) is computed as: ( ) =  −1 (1) ( ) (33) 

( −1)

Where  ( −1) is given by (20). k-nearest-neighbours of the query information vector are computed from the database. For this purpose, distance of the query vector to all the instances in the database is calculated using the ℒ -1 norm using (13). Suitable PID gains are computed from the parameters of k-nearest-neighbour of the query (14). After calculating the parameters from the database, they are updated using steepest gradient descent method for reducing the cost function J using (15).  

( ) = 1 ( )2  (34) 

2

( ) =  ( ) − ( )  (35) 

0 r

The gradients in (15) can be derived as following: 

15

( + 1) Δy0(t) (36) 

- − ( + 1)T(1)

( ) KI( )

( + 1) x0(t) (37) 

- ( + 1)T(1)

( ) KI( )2

( + 1) Δ2y0(t)

- − ( + 1)T(1) (38) 

( ) KI( )

The calculated  ( ) is updated into the database corresponding to the current query and the process is repeated at every instance of time for a number of epochs.  

After the database is updated with new PID parameters, we can calculate the output by getting the PID parameters from k-nearest-neighbours of the query.  

16
Simulation Example 

We will use the same Hammerstein model to see the difference of this model from conventional fixed PID controller. The following two systems are switched between during the runtime. 

System: 

`  `( ) = 0.6 ( − 1) − 0.1 ( − 2) + 1.2 ( − 1) − 0.1 ( − 2)  (39)   ( ) = 1.5 ( ) − 1.5 2( ) + 0.5 3( )  (40) 

The reference signal is given as: 

0.5(0 ≤ < 50)

( ) = { 1.0(50 ≤ < 100) (41) 

2.0(100 ≤ < 150)

1.5(150 ≤ ≤ 200)

The database is updated for 50 epochs. The following hyper-parameters were set for this example. 



|Orders of the information vector |Number of Neighbours |Learning Rates |Size of the Database |
| :- | - | - | - |
|<p>- 3 </p><p>- 2 </p>|= 6 |<p>- 0.1  </p><p>- 0.001 </p><p>- 0.1 </p>|= 200 |
|Fixed PID parameters|<p>KP = 0.486, KI = 0.227, K = 0.122</p><p>D</p>|
|Characteristic Polynomial Coefficients|t1 = -0.271, t2 = 0.0183 |

17
` `The simulation results are below: 

![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.009.jpeg) ![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.010.jpeg)

*Figure 7 Fixed PID output*  *Figure 8 Data Driven FRIT output* 

` `![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.011.jpeg)![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.012.jpeg)

*Figure 9 Trajectory of PID parameters*  *Figure 10 Comparison of both the outputs* 

![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.013.jpeg)

*Figure 11 Cost function over the epochs* 

19

Experiment on Tank System ![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.014.png)

The DD FRIT control method was used on a Tank System for level control. The tank number 2 has a nonlinear relation between its volume and the height, therefore this is a nonlinear system.   

![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.015.jpeg)

*Figure 12 Tank System*

The tank system being non-metallic cannot have input of hot water. Therefore, only cold flow was controlled to maintain a level specified by the reference signal. However, the system had a hysteresis which had to be removed before applying the Data Driven FRIT control scheme. For this, a local controller with the actuator was tuned to make the cold flow input and output be linear with each other.  

![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.016.jpeg)

*Figure 13 Local controller for hysteresis* 

20

The database is updated for 5 epochs. The following hyper-parameters were set for this example. 



|Orders of the information vector |Number of Neighbours |Learning Rates |Size of the Database |
| :- | - | - | - |
|<p>- 3 </p><p>- 2 </p>|= 6 |<p>- 0.003  </p><p>- 0.00005 </p><p>- 0 </p>|= 601 |
|Fixed PID parameters|<p>KP = 0.3, KI = 0.003, K = 0</p><p>D</p>|
|Characteristic Polynomial Coefficients|t1 = -1.9695, t2 = 0.9697|
After tuning the local PID controller, the main controller was tuned to give barely acceptable results as shown in the figure 14. The input to the system G(s) for the cold flow valve is in percentage of the valve opened. For the control system, the output from the main controller and local controller are saturated from 10% to 40%. This is because the system has the best performance in that range.  

We can notice that the input signal u(t) is not strong enough. A stronger input signal can reduce the rise time of the output signal. This is exactly what is  achieved by applying the data driven FRIT control scheme. In figure 17, the input signal can be compared to be stronger after getting the PID parameters from the DD FRIT control method. 

![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.017.jpeg) ![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.018.jpeg)

*Figure 14 Fixed PID output*  *Figure 15 Data Driven PID output* 

![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.019.jpeg) ![](Aspose.Words.8c2fa631-03b9-4b4a-bbd3-f754bacc33a8.020.jpeg)

*Figure 16 Trajectory of PID parameters*  *Figure 17 Comparison of both the outputs* 

In this experiment, however, the differential gain  KD is forced to be zero. This is because a differential block act as a high pass filter that allow high frequency noise to be amplified. Since the sensor for water level had a lot of noise component, it was better to remove the differential gain from the controller. Hence, KD in figure 16 is zero at all times.   

References

1. T. Yamamoto, K. Takao and T. Yamada, "Design of a Data-Driven PID Controller," in *IEEE Transactions on Control Systems Technology*, vol. 17, no. 1, pp. 29-39, Jan. 2009. doi: 10.1109/TCST.2008.921808 
1. Wakitani, Shin & Yamamoto, Toru & Gopaluni, Bhushan. (2019). Design and Application of a Database-Driven PID Controller with Data-Driven Updating Algorithm. Industrial & Engineering Chemistry Research. 10.1021/acs.iecr.9b00704. 
1. S. Wakitani and T. Yamamoto, "A learning algorithm for a data-driven controller based on fictitious reference iterative tuning," *2016 International Joint Conference on Neural Net- works (IJCNN)*, Vancouver, BC, 2016, pp. 4908-4913. doi: 10.1109/IJCNN.2016.7727845 
23

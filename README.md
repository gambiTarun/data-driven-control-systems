# data-driven-control-systems
Design of Data-Driven Controller and its applications

A REPORT

ON

DESIGN OF DATA-DRIVEN

CONTROLLER AND ITS APPLICATION

BY

TARUNBIR SINGH GAMBHIR

B.E (Hons.) Electrical and Electronics Engineering

BIRLA INSTITUTE OF TECHNOLOGY & SCIENCE, PILANI

AT

THE DEPARTMENT OF

CONTROL SYSTEMS ENGINEERING

HIROSHIMA UNIVERITY

JULY 2019


## Acknowledgements

I would like to thank **Prof. Toru Yamamoto** for giving me chance to work on this

research topic and gain experience employing this control scheme on a tank

system.

I would also like to thank **Dr. T.Kinoshita** for all his time and guidance for the

successful completion and implementation of the project.

I would like to thank **Keishi Fujiwara** for all his time and effort in helping me

understand basic concepts and clearing away my doubts.


## Abstract

Most of the real-world systems exhibit nonlinearity and are therefore governed

by nonlinear differential equations. Control design schemes that can deal with

such systems efficiently are designed and implemented in this project. A

conventional I-PD controller is modified to a data-driven controller such that the

PID parameters become dynamic in nature and can adapt to get the desired

system output. The idea is to use data collected from old experiments to

generate the PID parameters that can adapt to the nonlinearity in the output.

The change in PID parameters can be realised in an online manner, where the

size of the database is dynamic, or an offline manner, where the database is

fixed in size.


## Table of Contents

- Acknowledgements
- Abstract
- Introduction
- Design of DD PID Control
   - Simulation Example
- Design of DD FRIT Control
   - Simulation Example
   - Experiment on Tank System
- References


## Introduction

The real systems have nonlinearity that can be dealt to some extent using a PID

control. But systems with high nonlinearly cannot be controlled by a PID controller

with fixed parameters. Dynamic parameters allow for better control of systems with

strong nonlinearity. In this project, a technique for self-tuning PID control is designed

and implemented using recorded data of a system. However, for the controller to work

on a nonlinear system the data has be recorded from a nonlinear system.

In the past, many complex control algorithms using the aid of neural networks or

genetic algorithms have been proposed for systems with strong nonlinearity. They do

come with their drawbacks including high computational cost and large data

requirement.

An online data-driven control scheme is designed that reduces the data requirements

and thereby the computational costs. The PID parameters are adequately adapted in

proportion to the control errors, and the updated parameters are stored in the

database. The database is also stripped of any redundant data points in real-time to

reduce the computational and memory storage requirements.

Furthermore, an offline data-driven based fictious reference iterative tuning (DD FRIT)

control scheme is designed and implemented. In this algorithm, a reference signal is

generated from the initial closed-loop data and control parameters, and the optimal

PID parameters are calculated using this signal. The major difference from the online

method is that new data is not added to the database in real-time.


## Design of DD PID Control

The system is considered a discrete-time nonlinear system:

#### 𝑦

#### (

#### 𝑡

#### )

#### = 𝑓(𝜙

#### (

#### 𝑡− 1

#### )

#### )

#### ( 1 )

#### 𝜙(𝑡− 1 ) ≔ [𝑦

#### (

#### 𝑡− 1

#### )

#### ,...,𝑦(𝑡−𝑛

```
𝑦
```
#### ),𝑢

#### (

#### 𝑡− 1

#### )

#### ,...,𝑢(𝑡−𝑛

```
𝑢
```
#### )] ( 2 )

Where y(t) denotes system output and f(.) denotes the nonlinear function. Here, 𝜙(t)

denotes the ‘information vector’.

A conventional digital PID controller is given by:

#### ∆𝑢

#### (

#### 𝑘

#### )

#### =𝐾

```
𝑃
```
#### ∆𝑒

#### (

#### 𝑘

#### )

#### + 𝐾

```
𝐼
```
#### 𝑒

#### (

#### 𝑘

#### )

#### + 𝐾

```
𝐷
```
#### ∆

```
2
```
#### 𝑒(𝑘) ( 3 )

Where K denotes the PID parameters, Δ is the difference operator and e(k) is the

control error in the discrete-time system.

Using the fact that Δ

```
2
```
```
e(k) = e(k) – 2e(k-1) + e(k-2), and assuming that the reference
```
values are constant we can write the above equation as follows:

#### ∆𝑢(𝑡)=𝐾

```
𝐼
```
#### 𝑒(𝑡)− 𝐾

```
𝑃
```
#### ∆𝑦(𝑡)− 𝐾

```
𝐷
```
#### ∆

```
2
```
#### 𝑦(𝑡) ( 4 )

This PID control law is known as the I-PD control law. We shall be using the I-PD control

law in the designing of the data driven control scheme.

For systems with strong nonlinearities, it is hard to obtain good control with fixed PID

parameters. Therefore, time variant PID parameters are introduced in the following

control law:

#### ∆𝑢

#### (

#### 𝑡

#### )

#### =𝐾

```
𝐼
```
#### (𝑡)𝑒

#### (

#### 𝑡

#### )

#### − 𝐾

```
𝑃
```
#### (𝑡)∆𝑦

#### (

#### 𝑡

#### )

#### − 𝐾

```
𝐷
```
#### (𝑡)∆

```
2
```
#### 𝑦(𝑡) ( 5 )

u(t) can be rewritten as:

#### 𝑢

#### (

#### 𝑡

#### )

#### =𝑔(𝜙

```
′
```
#### (

#### 𝑡

#### )

#### ) ( 6 )

#### 𝜙′(𝑡) ≔ [𝐾

#### (

#### 𝑡

#### )

#### ,𝑟

#### (

#### 𝑡

#### )

#### ,𝑦

#### (

#### 𝑡

#### )

#### ,𝑦

#### (

#### 𝑡− 1

#### )

#### ,𝑦

#### (

#### 𝑡− 2

#### )

#### ,𝑢

#### (

#### 𝑡− 1

#### )

#### ] ( 7 )


#### 𝐾(𝑡)≔[𝐾

```
𝑃
```
#### (𝑡),𝐾

```
𝐼
```
#### (𝑡),𝐾

```
𝐷
```
#### (𝑡)] ( 8 )

Where g(.) denotes the function from (5). By substituting (6) and (7) into (1) and (2)

the following can be derived:

#### 𝐾(𝑡) = 𝐹(𝜙(𝑡)) ( 9 )

#### 𝜙

#### (

#### 𝑡

#### )

#### ∶=[𝑦

#### (

#### 𝑡+ 1

#### )

#### ,𝑦

#### (

#### 𝑡

#### )

#### ,...,𝑦(𝑡−𝑛

```
𝑦
```
#### + 1 ),

#### 𝑟(𝑡),𝑢(𝑡− 1 ),...,𝑢(𝑡−𝑛

```
𝑢
```
#### + 1 )]

#### ( 10 )

It is not possible to obtain future output y(t+1) at time t, so y(t+1) is replaced with

r(t+1) as the value of y(t+1) —> r(t+1) using a controller. Therefore,

#### 𝜙(𝑡)∶=[𝑟(𝑡+ 1 ),𝑦(𝑡),...,𝑦(𝑡−𝑛

```
𝑦
```
#### + 1 ),

#### 𝑟

#### (

#### 𝑡

#### )

#### ,𝑢

#### (

#### 𝑡− 1

#### )

#### ,...,𝑢

#### (

#### 𝑡−𝑛

```
𝑢
```
#### + 1

#### )

#### ]

#### ( 11 )

The initial database is created from instances of the information vector (9) generated

by the initial run with fixed PID parameters.

#### 𝛷

#### (

#### 𝑗

#### )

#### =

#### [

#### 𝜙

#### (

#### 𝑗

#### )

#### ,𝐾

#### (

#### 𝑗

#### )]

```
, j=1,2,...,N(0) ( 12 )
```
This database is used for PID parameter tuning in real-time. At every instance, k-

nearest-neighbours of the query information vector are computed from the database.

For this purpose, distance of the query vector to all the instances in the database is

calculated using the ℒ - 1 norm:

#### 𝑑(𝜙

#### (

#### 𝑡

#### )

#### ,𝜙

```
′
```
#### (

#### 𝑗

#### )

#### )= ∑ |

#### 𝜙

```
𝑙
```
#### (

#### 𝑡

#### )

#### −𝜙

```
𝑙
```
#### (

#### 𝑗

#### )

#### 𝑚𝑎𝑥

```
𝑚
```
#### 𝜙

```
𝑙
```
#### (𝑚)− 𝑚𝑖𝑛

```
𝑚
```
#### 𝜙

```
𝑙
```
#### (𝑚)

#### |

```
𝑛
𝑦
```
```
+ 𝑛
𝑢
```
```
+ 1
```
```
𝑙= 1
```
#### ,

```
j = 1,2,...,N(t)
```
#### ( 13 )

N(t) denotes the number of information vectors stored at that instance. Suitable PID

gains are computed from the parameters of k-nearest-neighbour of the query as

following:


#### 𝐾

```
𝑜𝑙𝑑
```
#### (

#### 𝑡

#### )

#### =

#### ∑

#### 𝑤

```
𝑖
```
```
𝑘
```
```
𝑖= 0
```
#### 𝐾

```
𝑃𝐼𝐷
```
#### (

#### 𝑡

```
𝑖
```
#### )

#### ,

#### ∑

#### 𝑤

```
𝑖
```
#### = 1

```
𝑘
```
```
𝑖= 1
```
#### ( 14 )

After calculating the parameters from the database, they are updated using steepest

gradient descent method for reducing the cost function J.

#### 𝐾

```
𝑛𝑒𝑤
```
#### (𝑡)= 𝐾

```
𝑜𝑙𝑑
```
#### (𝑡)− 𝜂

```
𝜕𝐽(𝑡+ 1 )
```
```
𝜕𝐾(𝑡)
```
#### ( 15 )

#### 𝜂≔[𝜂

```
𝑃
```
#### ,𝜂

```
𝐼
```
#### ,𝜂

```
𝐷
```
#### ]

#### ( 16 )

Where 𝜂 gives the learning rate of the gradient descent algorithm and J(t) is the cost

function defined by:

#### 𝐽(𝑡)=

#### 1

#### 2

#### 𝜀(𝑡)

```
2
```
#### ( 17 )

#### 𝜀

#### (

#### 𝑡

#### )

#### = 𝑦

```
𝑟
```
#### (

#### 𝑡

#### )

#### −𝑦(𝑡)

#### ( 18 )

#### 𝑦

```
𝑟
```
```
(t) denotes the output of the reference model and is defined by:
```
#### 𝑦

```
𝑟
```
#### (

#### 𝑡

#### )

#### =

#### 𝑧

```
− 1
```
#### 𝑇

#### (

#### 1

#### )

#### 𝑇

#### (

#### 𝑧

```
− 1
```
#### )

#### 𝑟(𝑡)

#### ( 19 )

#### 𝑇

#### (

#### 𝑧

```
− 1
```
#### )

#### = 1 +𝑡

```
1
```
#### 𝑧

```
− 1
```
#### +𝑡

```
2
```
#### 𝑧

```
− 2
```
#### ( 20 )

Here, 𝑇

#### (

#### 𝑧

```
− 1
```
#### )

```
is designed based on the desired rise-time and the damping property.
```
The gradients in (15) can be derived as following:

```
𝜕𝐽(𝑡+ 1 )
```
```
𝜕𝐾
```
```
𝑃
```
```
(𝑡)
```
#### = 𝜀(𝑡+ 1 )(𝑦

```
(
𝑡
```
```
)
−𝑦
```
```
(
𝑡− 1
```
```
)
)
```
```
𝜕𝑦(𝑡+ 1 )
```
```
𝜕𝑢(𝑡)
```
#### ( 21 )

```
𝜕𝐽(𝑡+ 1 )
```
```
𝜕𝐾
```
```
𝐼
```
```
(𝑡)
```
#### = −𝜀(𝑡+ 1 )(𝑟

```
(
𝑡
```
```
)
−𝑦
```
```
(
𝑡
```
```
)
)
```
```
𝜕𝑦(𝑡+ 1 )
```
```
𝜕𝑢(𝑡)
```
#### ( 22 )

```
𝜕𝐽(𝑡+ 1 )
```
```
𝜕𝐾
```
```
𝐷
```
```
(𝑡)
```
#### = 𝜀(𝑡+ 1 )(𝑦(𝑡)− 2 𝑦(𝑡− 1 )+𝑦(𝑡− 2 ))

```
𝜕𝑦(𝑡+ 1 )
```
```
𝜕𝑢(𝑡)
```
#### ( 23 )

The next step is to check for data redundancy and remove the instances in the

database that are similar. This can be done by setting a threshold distance below which


the instances in the database are considered redundant. In this case two conditions

are used to filter out the redundant data in the database. First condition,

#### 𝑑(𝜙

#### (

#### 𝑡

#### )

#### ,𝜙

```
′
```
#### (

#### 𝑖

#### )

#### )≤𝛼

```
1
```
#### , 𝑖= 1 , 2 ,...,𝑁

#### (

#### 𝑡

#### )

#### −𝑘. ( 24 )

extracts the instances in the databases that satisfy the above condition. Furthermore,

these information vectors are passed to the Second condition:

#### ∑{

#### 𝐾

```
𝑙
```
#### (𝑖)−𝐾

```
𝑙
```
```
𝑛𝑒𝑤
```
#### (𝑡)

#### 𝐾

```
𝑙
```
```
𝑛𝑒𝑤
```
#### (

#### 𝑡

#### )

#### }

```
2
```
#### ≤𝛼

```
2
```
```
3
```
```
𝑙= 1
```
#### ( 25 )

The vectors extracted are deleted from the database as they have high similarity with

the query information vector. The parameters 𝛼

```
1
```
```
and 𝛼
```
```
2
```
```
are set by trial and error.
```
The query information vector along with the calculated 𝐾

```
𝑛𝑒𝑤
```
```
(𝑡) is added into the
```
database and the process is repeated at every instance of time.


### Simulation Example

We will use a Hammerstein model to depict the usefulness of the data-driven control

method. The following two systems are switched between during the runtime.

System 1 (0<t<70)

#### 𝑦

#### (

#### 𝑡

#### )

#### = 0. 6 𝑦

#### (

#### 𝑡− 1

#### )

#### − 0. 1 𝑦

#### (

#### 𝑡− 2

#### )

#### + 1. 2 𝑥

#### (

#### 𝑡− 1

#### )

#### − 0. 1 𝑥(𝑡− 2 ) ( 26 )

#### 𝑥

#### (

#### 𝑡

#### )

#### = 1. 5 𝑢

#### (

#### 𝑡

#### )

#### − 1. 5 𝑢

```
2
```
#### (

#### 𝑡

#### )

#### − 0. 5 𝑢

```
3
```
#### (𝑡) ( 27 )

System 2 (70≤t< 200 )

#### 𝑦

#### (

#### 𝑡

#### )

#### = 0. 6 𝑦

#### (

#### 𝑡− 1

#### )

#### − 0. 1 𝑦

#### (

#### 𝑡− 2

#### )

#### + 1. 2 𝑥

#### (

#### 𝑡− 1

#### )

#### − 0. 1 𝑥(𝑡− 2 )

#### ( 28 )

#### 𝑥

#### (

#### 𝑡

#### )

#### = 1. 5 𝑢

#### (

#### 𝑡

#### )

#### − 1. 5 𝑢

```
2
```
#### (

#### 𝑡

#### )

#### − 1. 0 𝑢

```
3
```
#### (𝑡)

#### ( 29 )

The reference signal is given as:

#### 𝑟

#### (

#### 𝑡

#### )

#### ={

#### 0. 5

#### (

#### 0 ≤𝑡< 50

#### )

#### 1. 0

#### (

#### 50 ≤𝑡< 100

#### )

#### 2. 0 ( 100 ≤𝑡< 150 )

#### 1. 5 ( 150 ≤𝑡≤ 200 )

#### ( 30 )

The database is updated for 10 epochs. The following hyper-parameters were set for

this example.

```
Orders of the
```
```
information
```
```
vector
```
```
Number of
```
```
Neighbours
```
```
Learning Rates Coefficients to
```
```
remove
```
```
redundant data
```
```
Initial Number of
```
```
Data
```
```
𝑛
```
```
𝑦
```
```
= 3
```
```
𝑛
```
```
𝑢
```
```
= 2
```
```
𝑘= 6 𝜂
```
```
𝑃
```
```
= 0. 8
```
```
𝜂
```
```
𝐼
```
```
= 0. 8
```
```
𝜂
```
```
𝐷
```
```
= 0. 2
```
```
𝛼
```
```
1
```
```
= 0. 0005
```
```
𝛼
```
```
2
```
```
= 0. 0001
```
```
𝑁( 0 )= 6
```
```
Fixed PID parameters K
P
```
```
= 0.486, K
I
```
```
= 0.227, K
D
```
```
= 0.
```
```
Characteristic Polynomial Coefficients t
1
```
```
= -0.271, t
2
```
```
= 0.
```

The simulation results are below:

```
Figure 1 Fixed PID output
Figure 2 Data Driven PID output
```
```
Figure 3 Trajectory of PID parameters
```
```
Figure 4 Comparison of both the outputs
```
```
Figure 5 Vectors removed from the database
```
```
Figure 6 Cost function over the epochs
```

## Design of DD FRIT Control

The database is created in a similar way from instances of the information vector

generated by the run with fixed PID parameters. The database in this method is used

to find the optimal control parameter vector.

#### 𝛷

#### (

#### 𝑗

#### )

#### =

#### [

#### 𝜙

```
0
```
#### (

#### 𝑗

#### )

#### ,𝐾

```
0
```
#### (

#### 𝑗

#### )]

```
, j=1,2,...,N ( 31 )
```
Where N is the size of the database. The database is size not increased with every

query as it was the case with DD PID control scheme.

A fictious reference signal is generated from the closed loop data and the control

parameters in the database. The value of this reference signal can be derived as:

```
r̃(t) = y
```
```
0
```
```
(t) +
```
#### 1

#### K

```
I
```
```
(t)
```
```
{Δu
```
```
0
```
```
(t)+K
```
```
P
```
```
(t)Δy
```
```
0
```
```
(t)+K
```
```
D
```
```
(t)Δ
```
```
2
```
```
y
```
```
0
```
```
(t)} ( 32 )
```
After getting the reference signal, the output reference signal 𝑦

```
𝑟
```
#### (

#### 𝑡

#### )

```
is computed as:
```
#### 𝑦

```
𝑟
```
#### (

#### 𝑡

#### )

#### =

#### 𝑧

```
− 1
```
#### 𝑇

#### (

#### 1

#### )

#### 𝑇

#### (

#### 𝑧

```
− 1
```
#### )

#### 𝑟̃(𝑡)

#### ( 33 )

Where 𝑇

#### (

#### 𝑧

```
− 1
```
#### )

```
is given by (20). k-nearest-neighbours of the query information vector
```
are computed from the database. For this purpose, distance of the query vector to all

the instances in the database is calculated using the ℒ - 1 norm using (13). Suitable PID

gains are computed from the parameters of k-nearest-neighbour of the query (14).

After calculating the parameters from the database, they are updated using steepest

gradient descent method for reducing the cost function J using (15).

#### 𝐽

#### (

#### 𝑡

#### )

#### =

#### 1

#### 2

#### 𝜀(𝑡)

```
2
```
#### ( 34 )

#### 𝜀

#### (

#### 𝑡

#### )

#### = 𝑦

```
0
```
#### (

#### 𝑡

#### )

#### −𝑦

```
r
```
#### (𝑡) ( 35 )

The gradients in (15) can be derived as following:


```
𝜕𝐽(𝑡+ 1 )
```
```
𝜕𝐾
```
```
𝑃
```
```
(𝑡)
```
#### = −𝜀(𝑡+ 1 )T( 1 )

```
Δy
```
```
0
```
```
(t)
```
```
K
```
```
I
```
```
(𝑡)
```
#### ( 36 )

```
𝜕𝐽(𝑡+ 1 )
```
```
𝜕𝐾
```
```
𝐼
```
```
(𝑡)
```
#### = 𝜀(𝑡+ 1 )T( 1 )

```
x
```
```
0
```
```
(t)
```
```
K
```
```
I
```
```
(𝑡)
```
```
2
```
#### ( 37 )

```
𝜕𝐽(𝑡+ 1 )
```
```
𝜕𝐾
```
```
𝐷
```
```
(𝑡)
```
#### = −𝜀(𝑡+ 1 )T( 1 )

```
Δ
```
```
2
```
```
y
```
```
0
```
```
(t)
```
```
K
```
```
I
```
```
(𝑡)
```
#### ( 38 )

The calculated 𝐾

```
𝑛𝑒𝑤
```
#### (

#### 𝑡

#### )

```
is updated into the database corresponding to the current
```
query and the process is repeated at every instance of time for a number of epochs.

After the database is updated with new PID parameters, we can calculate the output

by getting the PID parameters from k-nearest-neighbours of the query.


### Simulation Example

We will use the same Hammerstein model to see the difference of this model from

conventional fixed PID controller. The following two systems are switched between

during the runtime.

System:

#### 𝑦

#### (

#### 𝑡

#### )

#### = 0. 6 𝑦

#### (

#### 𝑡− 1

#### )

#### − 0. 1 𝑦

#### (

#### 𝑡− 2

#### )

#### + 1. 2 𝑥

#### (

#### 𝑡− 1

#### )

#### − 0. 1 𝑥(𝑡− 2 ) ( 39 )

#### 𝑥

#### (

#### 𝑡

#### )

#### = 1. 5 𝑢

#### (

#### 𝑡

#### )

#### − 1. 5 𝑢

```
2
```
#### (

#### 𝑡

#### )

#### + 0. 5 𝑢

```
3
```
#### (𝑡) ( 40 )

The reference signal is given as:

#### 𝑟

#### (

#### 𝑡

#### )

#### ={

#### 0. 5

#### (

#### 0 ≤𝑡< 50

#### )

#### 1. 0

#### (

#### 50 ≤𝑡< 100

#### )

#### 2. 0 ( 100 ≤𝑡< 150 )

#### 1. 5 ( 150 ≤𝑡≤ 200 )

#### (4 1 )

The database is updated for 5 0 epochs. The following hyper-parameters were set for

this example.

```
Orders of the
```
```
information vector
```
```
Number of
```
```
Neighbours
```
```
Learning Rates Size of the
```
```
Database
```
#### 𝑛

```
𝑦
```
#### = 3

#### 𝑛

```
𝑢
```
#### = 2

#### 𝑘= 6 𝜂

```
𝑃
```
#### = 0. 1

#### 𝜂

```
𝐼
```
#### = 0. 001

#### 𝜂

```
𝐷
```
#### = 0. 1

#### 𝑁= 200

```
Fixed PID parameters K
P
```
```
= 0.486, K
I
```
```
= 0.227, K
D
```
#### = 0.

```
Characteristic Polynomial Coefficients t
1
```
```
= -0.271, t
2
```
#### = 0.


The simulation results are below:

```
Figure 7 Fixed PID output
Figure 8 Data Driven FRIT output
```
```
Figure 9 Trajectory of PID parameters
```
```
Figure 10 Comparison of both the outputs
```
```
Figure 11 Cost function over the epochs
```

### Experiment on Tank System

The DD FRIT control method was used on a Tank System for level control. The tank

number 2 has a nonlinear relation between its volume and the height, therefore this

is a nonlinear system.

```
Figure 12 Tank System
```
The tank system being non-metallic cannot have input of hot water. Therefore, only

cold flow was controlled to maintain a level specified by the reference signal. However,

the system had a hysteresis which had to be removed before applying the Data Driven

FRIT control scheme. For this, a local controller with the actuator was tuned to make

the cold flow input and output be linear with each other.

```
Figure 13 Local controller for hysteresis
```

The database is updated for 5 epochs. The following hyper-parameters were set for

this example.

```
Orders of the
```
```
information vector
```
```
Number of
```
```
Neighbours
```
```
Learning Rates Size of the
```
```
Database
```
#### 𝑛

```
𝑦
```
#### = 3

#### 𝑛

```
𝑢
```
#### = 2

#### 𝑘= 6 𝜂

```
𝑃
```
#### = 0. 003

#### 𝜂

```
𝐼
```
#### = 0. 00005

#### 𝜂

```
𝐷
```
#### = 0

#### 𝑁= 601

```
Fixed PID parameters K
P
```
```
= 0. 3 , K
I
```
```
= 0. 003 , K
D
```
#### = 0

```
Characteristic Polynomial Coefficients t
1
```
```
= -1.9695, t
2
```
#### = 0.

After tuning the local PID controller, the main controller was tuned to give barely

```
acceptable results as shown in the figure 14. The input to the system G(s) for the cold
```
```
flow valve is in percentage of the valve opened. For the control system, the output
```
```
from the main controller and local controller are saturated from 10% to 40%. This is
```
```
because the system has the best performance in that range.
```
We can notice that the input signal u(t) is not strong enough. A stronger input signal

```
can reduce the rise time of the output signal. This is exactly what is achieved by
```
```
applying the data driven FRIT control scheme. In figure 17, the input signal can be
```
```
compared to be stronger after getting the PID parameters from the DD FRIT control
```
```
method.
```

```
Figure 14 Fixed PID output
Figure 15 Data Driven PID output
```
```
Figure 16 Trajectory of PID parameters
Figure 17 Comparison of both the outputs
```
In this experiment, however, the differential gain K
D

```
is forced to be zero. This is
```
because a differential block act as a high pass filter that allow high frequency noise to

be amplified. Since the sensor for water level had a lot of noise component, it was

better to remove the differential gain from the controller. Hence, K
D

```
in figure 16 is zero
```
at all times.


## References

[1] T. Yamamoto, K. Takao and T. Yamada, "Design of a Data-Driven PID Controller,"

```
in IEEE Transactions on Control Systems Technology, vol. 17, no. 1, pp. 29-39, Jan.
```
2009. doi: 10.1109/TCST.2008.

[ 2 ] Wakitani, Shin & Yamamoto, Toru & Gopaluni, Bhushan. (2019). Design and Application

```
of a Database-Driven PID Controller with Data-Driven Updating Algorithm. Industrial &
```
```
Engineering Chemistry Research. 10.1021/acs.iecr.9b00704.
```
[ 3 ] S. Wakitani and T. Yamamoto, "A learning algorithm for a data-driven controller based on

```
fictitious reference iterative tuning," 2016 International Joint Conference on Neural Net-
```
```
works (IJCNN), Vancouver, BC, 2016, pp. 4908-4913. doi:
```
```
10.1109/IJCNN.2016.
```


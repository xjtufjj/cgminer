###Protocol

T:CMD Type

L:Data Len

D:Data(NA when L=0)

| Field         | Length(Byte)  | 
| :------------ |--------------:| 
| CMD Type      |             1 | 
| Data Len      |             1 | 
| Data          |      Data Len | 

###Work Flow

M:cgminer

S:BE200 Blade

* M send Self Test, S send result
* M send Job to S, S start to dispatch Job to chips by increasing nTime; if Job is not clean, S should discard current Job and switch to new Job immediately
* S collect nonces from chips and send back to M
* M check nonce and calculate hash rate for each chip and reset any chip or tweak freq for that chip if necessary

###Command

* Self Test

> Send: T=1; L=0; D=NA
> Recv: T=1; L=N; D=Self Test Result

* Reset Chip

> Send: T=2; L=Chip Count; D=[Chip ID]

* Tweak Chip Freq

> Send: T=3; L=2; D:Chip ID::Freq 

* Send Job

> Send: T=4; L=45; D=Job ID::44 Byte Work(Job is clean)
> Send: T=5; L=45; D=Job ID::44 Byte Work(Job is not clean)

* Receive Nonce

> Recv: T=6; L=7; D=Job ID::Chip ID::Ntime Roll::4 Byte Nonce

* Request Job
> Recv: T=7; L=0; D=NA





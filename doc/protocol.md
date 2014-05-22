###Protocol

T:CMD Type

L:Data Len

D:Data(NA when L=0)

| Field         | Length(Byte)  | 
| :------------ |--------------:| 
| CMD Type      |             1 | 
| Data Len      |             1 | 
| Data          |      Data Len | 

###Command

* Self Test
> T=1 L=0 D=NA
* Reset Chip
> T=2 L=Chip Count D=[Chip ID]
* Send Job
> T=3 L=45 D=Job ID::44 Byte Work
* Receive Nonce
> T=4 L=6  D=Job ID::Ntime Roll::4 Byte Nonce


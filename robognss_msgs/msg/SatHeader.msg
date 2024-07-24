uint8   UNKNOWN=0
uint8   GPS=1   # 0x01
uint8   GAL=8   # 0x08
uint8   GLO=4   # 0x04
uint8   BDS=32  # 0x20
uint8   SBAS=2  # 0x02
uint8   QZSS=16 # 0x10
uint8                 sat_sys
string                sat_name

# freq code same as rtklib
uint8 CODE_NONE=0                  #/* obs code: none or unknown 
uint8 CODE_L1C=1                   #/* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) 
uint8 CODE_L1P=2                   #/* obs code: L1P,G1P,B1P (GPS,GLO,BDS) 
uint8 CODE_L1W=3                   #/* obs code: L1 Z-track (GPS) 
uint8 CODE_L1Y=4                   #/* obs code: L1Y        (GPS) 
uint8 CODE_L1M=5                   #/* obs code: L1M        (GPS) 
uint8 CODE_L1N=6                   #/* obs code: L1codeless,B1codeless (GPS,BDS) 
uint8 CODE_L1S=7                   #/* obs code: L1C(D)     (GPS,QZS) 
uint8 CODE_L1L=8                   #/* obs code: L1C(P)     (GPS,QZS) 
uint8 CODE_L1E=9                   #/* (not used) 
uint8 CODE_L1A=10                  #/* obs code: E1A,B1A    (GAL,BDS) 
uint8 CODE_L1B=11                  #/* obs code: E1B        (GAL) 
uint8 CODE_L1X=12                  #/* obs code: E1B+C,L1C(D+P),B1D+P (GAL,QZS,BDS) 
uint8 CODE_L1Z=13                  #/* obs code: E1A+B+C,L1S (GAL,QZS) 
uint8 CODE_L2C=14                  #/* obs code: L2C/A,G1C/A (GPS,GLO) 
uint8 CODE_L2D=15                  #/* obs code: L2 L1C/A-(P2-P1) (GPS) 
uint8 CODE_L2S=16                  #/* obs code: L2C(M)     (GPS,QZS) 
uint8 CODE_L2L=17                  #/* obs code: L2C(L)     (GPS,QZS) 
uint8 CODE_L2X=18                  #/* obs code: L2C(M+L),B1_2I+Q (GPS,QZS,BDS) 
uint8 CODE_L2P=19                  #/* obs code: L2P,G2P    (GPS,GLO) 
uint8 CODE_L2W=20                  #/* obs code: L2 Z-track (GPS) 
uint8 CODE_L2Y=21                  #/* obs code: L2Y        (GPS) 
uint8 CODE_L2M=22                  #/* obs code: L2M        (GPS) 
uint8 CODE_L2N=23                  #/* obs code: L2codeless (GPS) 
uint8 CODE_L5I=24                  #/* obs code: L5I,E5aI   (GPS,GAL,QZS,SBS) 
uint8 CODE_L5Q=25                  #/* obs code: L5Q,E5aQ   (GPS,GAL,QZS,SBS) 
uint8 CODE_L5X=26                  #/* obs code: L5I+Q,E5aI+Q,L5B+C,B2aD+P (GPS,GAL,QZS,IRN,SBS,BDS) 
uint8 CODE_L7I=27                  #/* obs code: E5bI,B2bI  (GAL,BDS) 
uint8 CODE_L7Q=28                  #/* obs code: E5bQ,B2bQ  (GAL,BDS) 
uint8 CODE_L7X=29                  #/* obs code: E5bI+Q,B2bI+Q (GAL,BDS) 
uint8 CODE_L6A=30                  #/* obs code: E6A,B3A    (GAL,BDS) 
uint8 CODE_L6B=31                  #/* obs code: E6B        (GAL) 
uint8 CODE_L6C=32                  #/* obs code: E6C        (GAL) 
uint8 CODE_L6X=33                  #/* obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,BDS) 
uint8 CODE_L6Z=34                  #/* obs code: E6A+B+C,L6D+E (GAL,QZS) 
uint8 CODE_L6S=35                  #/* obs code: L6S        (QZS) 
uint8 CODE_L6L=36                  #/* obs code: L6L        (QZS) 
uint8 CODE_L8I=37                  #/* obs code: E5abI      (GAL) 
uint8 CODE_L8Q=38                  #/* obs code: E5abQ      (GAL) 
uint8 CODE_L8X=39                  #/* obs code: E5abI+Q,B2abD+P (GAL,BDS) 
uint8 CODE_L2I=40                  #/* obs code: B1_2I      (BDS) 
uint8 CODE_L2Q=41                  #/* obs code: B1_2Q      (BDS) 
uint8 CODE_L6I=42                  #/* obs code: B3I        (BDS) 
uint8 CODE_L6Q=43                  #/* obs code: B3Q        (BDS) 
uint8 CODE_L3I=44                  #/* obs code: G3I        (GLO) 
uint8 CODE_L3Q=45                  #/* obs code: G3Q        (GLO) 
uint8 CODE_L3X=46                  #/* obs code: G3I+Q      (GLO) 
uint8 CODE_L1I=47                  #/* obs code: B1I        (BDS) (obsolete) 
uint8 CODE_L1Q=48                  #/* obs code: B1Q        (BDS) (obsolete) 
uint8 CODE_L5A=49                  #/* obs code: L5A SPS    (IRN) 
uint8 CODE_L5B=50                  #/* obs code: L5B RS(D)  (IRN) 
uint8 CODE_L5C=51                  #/* obs code: L5C RS(P)  (IRN) 
uint8 CODE_L9A=52                  #/* obs code: SA SPS     (IRN) 
uint8 CODE_L9B=53                  #/* obs code: SB RS(D)   (IRN) 
uint8 CODE_L9C=54                  #/* obs code: SC RS(P)   (IRN) 
uint8 CODE_L9X=55                  #/* obs code: SB+C       (IRN) 
uint8 CODE_L1D=56                  #/* obs code: B1D        (BDS) 
uint8 CODE_L5D=57                  #/* obs code: L5D(L5S),B2aD (QZS,BDS) 
uint8 CODE_L5P=58                  #/* obs code: L5P(L5S),B2aP (QZS,BDS) 
uint8 CODE_L5Z=59                  #/* obs code: L5D+P(L5S) (QZS) 
uint8 CODE_L6E=60                  #/* obs code: L6E        (QZS) 
uint8 CODE_L7D=61                  #/* obs code: B2bD       (BDS) 
uint8 CODE_L7P=62                  #/* obs code: B2bP       (BDS) 
uint8 CODE_L7Z=63                  #/* obs code: B2bD+P     (BDS) 
uint8 CODE_L8D=64                  #/* obs code: B2abD      (BDS) 
uint8 CODE_L8P=65                  #/* obs code: B2abP      (BDS) 
uint8 CODE_L4A=66                  #/* obs code: G1aL1OCd   (GLO) 
uint8 CODE_L4B=67                  #/* obs code: G1aL1OCd   (GLO) 
uint8 CODE_L4X=68                  #/* obs code: G1al1OCd+p (GLO) 
uint8                 freq_code
string                freq_name

uint16                sv_id



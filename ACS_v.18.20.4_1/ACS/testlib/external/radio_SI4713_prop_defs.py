# PTY codes & PTYN values based on RDS standard
PTY_rds = {
    1:"News",
    2:"Current affairs",
    3:"Information",
    4:"Sport",
    5:"Education",
    6:"Drama",
    7:"Culture",
    8:"Science",
    9:"Varied",
    10:"Pop Music",
    11:"Rock Music",
    12:"M.O.R. Music",
    13:"Light classical",
    14:"Serious classical",
    15:"Other Music",
    16:"Weather",
    17:"Finance",
    18:"Children programs",
    19:"Social Affairs",
    20:"Religion",
    21:"Phone In",
    22:"Travel 	Public",
    23:"Leisure",
    24:"Jazz Music",
    25:"Country Music",
    26:"National Music",
    27:"Oldies Music",
    28:"Folk Music",
    29:"Documentary",
    30:"Alarm Test",
    31:"Alarm"
}

# PTY codes & PTYN values based on RBDS standard
PTY_rbds = {0: None,
           1: "news",
           2: "information",
           3: "sports",
           4: "talk",
           5: "rock",
           6: "clasic rock",
           7: "adult hits",
           8: "soft rock",
           9: "top 40",
          10: "country",
          11: "oldies",
          12: "soft",
          13: "nostalgia",
          14: "jazz",
          15: "classical",
          16: "rhythm and blues",
          17: "soft rhythm and blues",
          18: "language",
          19: "religious music",
          20: "religious talk",
          21: "personality",
          22: "public",
          23: "college",
          24: None ,
          25: None,
          26: None,
          27: None,
          28: None,
          29: "weather",
          30: "emergency test",
          31: "emergency"
          }

TX_RDS_PS_MISC = { # the default, as per datasheet, value of this property
    15:'0',# RDSD3
    14:'0',# RDSD2
    13:'0',# RDSD1
    12:'1',# RDSD0
    11:'1', # FORCEB
    10:'0', # RDSTP
    9:'0', # RDSPTY:[4:0]
    8:'0', # RDSPTY:[4:0]
    7:'0', # RDSPTY:[4:0]
    6:'0', # RDSPTY:[4:0]
    5:'0', # RDSPTY:[4:0]
    4:'0', # RDSTA
    3:'1', # RDSMS
    2:'0', # Reserved [2:0]
    1:'0', # Reserved [2:0]
    0:'0' # Reserved [2:0]
}

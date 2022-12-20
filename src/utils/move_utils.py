MOVE_DICT= {
    "a1":[1, 65, 37, 57, 69],
    "a2":[0, 0, 0, 0, 0],
    "a3":[0, 0, 0, 0, 0],
    "a4":[0, 0, 0, 0, 0],
    "a5":[0, 0, 0, 0, 0],
    "a6":[0, 0, 0, 0, 0],
    "a7":[0, 0, 0, 0, 0],
    "a8":[0, 0, 0, 0, 0],
    "b1":[1, 65, 29, 65, 77],
    "b2":[0, 0, 0, 0, 0],
    "b3":[0, 0, 0, 0, 0],
    "b4":[0, 0, 0, 0, 0],
    "b5":[0, 0, 0, 0, 0],
    "b6":[0, 0, 0, 0, 0],
    "b7":[0, 0, 0, 0, 0],
    "b8":[0, 0, 0, 0, 0],
    "c1":[0, 0, 0, 0, 0],
    "c2":[0, 0, 0, 0, 0],
    "c3":[0, 0, 0, 0, 0],
    "c4":[0, 0, 0, 0, 0],
    "c5":[0, 0, 0, 0, 0],
    "c6":[0, 0, 0, 0, 0],
    "c7":[0, 0, 0, 0, 0],
    "c8":[0, 0, 0, 0, 0],
    "d1":[0, 0, 9, 77, 113],
    "d2":[0, 0, 0, 0, 0],
    "d3":[0, 0, 0, 0, 0],
    "d4":[153, 17, 25, 73, 97],
    "d5":[0, 0, 0, 0, 0],
    "d6":[0, 0, 0, 0, 0],
    "d7":[0, 0, 0, 0, 0],
    "d8":[0, 0, 0, 0, 0],
    "e1":[0, 0, 0, 69, 97],
    "e2":[0, 0, 0, 0, 0],
    "e3":[0, 0, 0, 0, 0],
    "e4":[5, 21, 29, 81, 101],
    "e5":[153, 17, 13, 77, 105],
    "e6":[0, 0, 0, 0, 0],
    "e7":[153, 0, 17, 65, 105],
    "e8":[0, 0, 0, 0, 0],
    "f1":[0, 0, 0, 69, 85],
    "f2":[0, 0, 0, 0, 0],
    "f3":[0, 0, 0, 0, 0],
    "f4":[0, 0, 0, 0, 0],
    "f5":[0, 0, 0, 0, 0],
    "f6":[0, 0, 0, 0, 0],
    "f7":[0, 0, 0, 0, 0],
    "f8":[0, 0, 0, 0, 0],
    "g1":[0, 0, 13, 65, 69],
    "g2":[0, 0, 0, 0, 0],
    "g3":[0, 0, 0, 0, 0],
    "g4":[0, 0, 0, 0, 0],
    "g5":[0, 0, 0, 0, 0],
    "g6":[0, 0, 0, 0, 0],
    "g7":[0, 0, 0, 0, 0],
    "g8":[0, 0, 0, 0, 0],
    "h1":[0, 0, 21, 69, 61],
    "h2":[0, 0, 0, 0, 0],
    "h3":[0, 0, 0, 0, 0],
    "h4":[0, 0, 0, 0, 0],
    "h5":[0, 0, 0, 0, 0],
    "h6":[0, 0, 0, 0, 0],
    "h7":[0, 0, 0, 0, 0],
    "h8":[0, 0, 0, 0, 0]
}
SAFE_POSE= [145, 0, 90, 0, 180, 90]
YEET_POSE= [49, 17, 25, 60, 165]
#e7e5 f8c5 d8f6 g7g5 f6f2


MANY_MOVES_DICT= {
    "a1":[[1, 65, 37, 57, 69]],
    "a2":[[0, 0, 0, 0, 0]],
    "a3":[[0, 0, 0, 0, 0]],
    "a4":[[0, 0, 0, 0, 0]],
    "a5":[[0, 0, 0, 0, 0]],
    "a6":[[0, 0, 0, 0, 0]],
    "a7":[[0, 0, 0, 0, 0]],
    "a8":[[0, 0, 0, 0, 0]],
    "b1":[[1, 65, 29, 65, 77]],
    "b2":[[0, 0, 0, 0, 0]],
    "b3":[[0, 0, 0, 0, 0]],
    "b4":[[0, 0, 0, 0, 0]],
    "b5":[[0, 0, 0, 0, 0]],
    "b6":[[0, 0, 0, 0, 0]],
    "b7":[[0, 0, 0, 0, 0]],
    "b8":[[0, 0, 0, 0, 0]],
    "c1":[[0, 0, 0, 0, 0]],
    "c2":[[0, 0, 0, 0, 0]],
    "c3":[[0, 0, 0, 0, 0]],
    "c4":[[0, 0, 0, 0, 0]],
    "c5":[
         # 0, 90, 90, 90, 90
        [SAFE_POSE[1], SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 82],
        [1, SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 82],
        [1, 21, SAFE_POSE[3], SAFE_POSE[4], 82],
        [1, 21, SAFE_POSE[3], 120, 82],

        [0, 21, 50, 120, 82],
        [0, 21, 50, 66, 82],
        [0, 27, 50, 66, 82],
        [0, 27, 45, 66, 82],
        [0, 27, 45, 62, 82]
    ],
    "c6":[[0, 0, 0, 0, 0]],
    "c7":[[0, 0, 0, 0, 0]],
    "c8":[[0, 0, 0, 0, 0]],
    "d1":[[0, 0, 9, 77, 113]],
    "d2":[[0, 0, 0, 0, 0]],
    "d3":[[0, 0, 0, 0, 0]],
    "d4":[[153, 17, 25, 73, 97]],
    "d5":[[0, 0, 0, 0, 0]],
    "d6":[[0, 0, 0, 0, 0]],
    "d7":[[0, 0, 0, 0, 0]],
    "d8":[
         # 0, 90, 90, 90, 90
        [SAFE_POSE[1], SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 88],
        [180, SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 88],
        [180, 1, SAFE_POSE[3], SAFE_POSE[4], 88],
        [180, 1, SAFE_POSE[3], 115, 88],

        [180, 1, 17, 115, 88],
        [180, 1, 17, 82, 88],
        [180, 1, 21, 75, 88],
        [180, 1, 21, 69, 88],
        [180, 1, 23, 69, 88],
        [180, 1, 27, 75, 88],
        [180, 1, 27, 75, 88]
    ],
    "e1":[[0, 0, 0, 69, 97]],
    "e2":[[0, 0, 0, 0, 0]],
    "e3":[[0, 0, 0, 0, 0]],
    "e4":[[5, 21, 29, 81, 101]],
    "e5":[
         # 0, 90, 90, 90, 90
        [SAFE_POSE[1], SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 98],
        [1, SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 98],
        [1, 20, SAFE_POSE[3], SAFE_POSE[4], 98],
        [1, 20, SAFE_POSE[3], 140, 98],

        [1, 21, 44, 140, 98],
        [1, 21, 44, 81, 98],
        [1, 25, 44, 81, 98],
        [1, 25, 44, 77, 98],
        [1, 25, 44, 77, 98],
        [1, 25, 44, 70, 98],
        [1, 26, 41, 70, 98],
        [1, 26, 41, 70, 98]
        ],
    "e6":[[0, 0, 0, 0, 0]],
    "e7":[
         # 0, 90, 90, 90, 90
        [SAFE_POSE[1], SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 100],
        [1, SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 100],
        [1, 24, SAFE_POSE[3], SAFE_POSE[4], 100],
        [1, 24, SAFE_POSE[3], 115, 100],

        [1, 24, 2, 115, 100],
        [1, 24, 2, 97, 100],
        [1, 28, 2, 97, 100],
        [1, 28, 2, 90, 100],
        [1, 28, 8, 90, 100],
        [1, 28, 8, 90, 100]
    ],
    "e8":[[0, 0, 0, 0, 0]],
    "f1":[[0, 0, 0, 69, 85]],
    "f2":[
         # 0, 90, 90, 90, 90
        [SAFE_POSE[1], SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 105],
        [1, SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 105],
        [1, 59, SAFE_POSE[3], SAFE_POSE[4], 105],
        [1, 59, SAFE_POSE[3], 115, 105],

        [1, 59, 70, 115, 105],
        [1, 59, 70, 80, 105],
        [1, 59, 70, 44, 105]
    ],
    "f22":[
         # 0, 90, 90, 90, 90
        [SAFE_POSE[1], SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 105],
        [1, SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 105],
        [1, 59, SAFE_POSE[3], SAFE_POSE[4], 105],
        [1, 59, SAFE_POSE[3], 115, 105],
        [1, 59, 78, 115, 105],
        [1, 59, 78, 44, 105],
        [1, 51, 78, 44, 105],
        [1, 51, 82, 44, 105],
        [1, 43, 82, 44, 105],
        [1, 43, 86, 44, 105],
        [1, 39, 86, 44, 105],
        [1, 39, 86, 44, 105],
    ],
    "f3":[[0, 0, 0, 0, 0]],
    "f4":[[0, 0, 0, 0, 0]],
    "f5":[[1, 25, 47, 67, 108]],
    "f6":[
         # 0, 90, 90, 90, 90
        [SAFE_POSE[1], SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 110],
        [1, SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 110],
        [1, 25, SAFE_POSE[3], SAFE_POSE[4], 110],
        [1, 25, SAFE_POSE[3], 115, 110],

        [1, 25, 20, 115, 110],
        [1, 25, 20, 100, 110],
        [1, 25, 30, 100, 110],
        [1, 25, 27, 88, 110],
        [1, 25, 27, 88, 110],
        [1, 26, 24, 88, 110],
        [1, 26, 28, 79, 110],
        [1, 30, 28, 72, 110],
        [1, 30, 28, 72, 110]
        ],
    "f7":[[0, 0, 0, 0, 0]],
    "f8":[
         # 0, 90, 90, 90, 90
        [SAFE_POSE[1], SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 115],
        [1, SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 115],
        [1, 1, SAFE_POSE[3], SAFE_POSE[4], 115],
        [1, 1, SAFE_POSE[3], 115, 115],

        [1, 1, 13, 115, 115],
        [1, 1, 13, 93, 115],
        [1, 1, 17, 88, 115],
        [1, 1, 21, 80, 115],
        [1, 1, 21, 77, 115],
        [1, 1, 25, 75, 115],
        [1, 1, 25, 67, 115],
        [1, 1, 29, 66, 115],
        [1, 1, 32, 69, 115],
        [1, 1, 32, 69, 115]
    ],
    "g1":[[0, 0, 13, 65, 69]],
    "g2":[[0, 0, 0, 0, 0]],
    "g3":[[0, 0, 0, 0, 0]],
    "g4":[[0, 0, 0, 0, 0]],
    "g5":[
         # 0, 90, 90, 90, 90
        [SAFE_POSE[1], SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 116],
        [13, SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 116],
        [13, 9, SAFE_POSE[3], SAFE_POSE[4], 116],
        [13, 9, SAFE_POSE[3], 115, 116],

        [13, 9, 61, 115, 116],
        [13, 9, 61, 82, 116],
        [13, 9, 61, 70, 116],
        [13, 14, 61, 70, 116],
        [13, 14, 61, 62, 116],
        [13, 21, 61, 62, 116],
        [13, 23, 54, 62, 116],
        [13, 23, 51, 58, 116]
    ],
    "g6":[[0, 0, 0, 0, 0]],
    "g7":[
         # 0, 90, 90, 90, 90
        [SAFE_POSE[1], SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 122],
        [17, SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 122],
        [17, 24, SAFE_POSE[3], SAFE_POSE[4], 122],
        [17, 24, SAFE_POSE[3], 115, 122],

        [17, 24, 15, 115, 122],
        [17, 24, 15, 85, 122],
        [17, 24, 15, 77, 122],
        [17, 24, 17, 83, 122],
        [17, 30, 17, 77, 122],
        [17, 30, 17,  77, 122]
    ],
    "g8":[[0, 0, 0, 0, 0]],
    "h1":[[0, 0, 21, 69, 61]],
    "h2":[[0, 0, 0, 0, 0]],
    "h3":[[0, 0, 0, 0, 0]],
    "h4":[[0, 0, 0, 0, 0]],
    "h5":[[0, 0, 0, 0, 0]],
    "h6":[[0, 0, 0, 0, 0]],
    "h7":[[0, 0, 0, 0, 0]],
    "h8":[[0, 0, 0, 0, 0]]
}
YEET_POSE_SEQUENCE= [
     # 0, 90, 90, 90, 90
    [SAFE_POSE[1], SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 161],
    [49, SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], 161],
    [49, 41, SAFE_POSE[3], SAFE_POSE[4], 161],
    [59, 41, SAFE_POSE[3], 130, 161],
    [59, 41, 25, 130, 161],
    [49, 41, 25, 65, 161],
    [49, 41, 25, 65, 161]
]
DANCE_POSE_SEQUENCE= [
    [100,SAFE_POSE[1], SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], SAFE_POSE[5]],
    [180, 180, 120, SAFE_POSE[3], SAFE_POSE[4], 130],
    [100, 0, 70, SAFE_POSE[3], SAFE_POSE[4], 50],
    [180, 180, 120, SAFE_POSE[3], SAFE_POSE[4], 130],
    [100, 0, 70, SAFE_POSE[3], SAFE_POSE[4], 50],
    [180, 180, 120, SAFE_POSE[3], SAFE_POSE[4], 130],
    [100,SAFE_POSE[1], SAFE_POSE[2], SAFE_POSE[3], SAFE_POSE[4], SAFE_POSE[5]],
]

#e7e5 f8c5 d8f6 g7g5 f6f2
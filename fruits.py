'''
Source code for Fruit Ninja AI ( https://www.youtube.com/watch?v=Vw3vU9OdWAs ) 

The AI only loses when a bomb is overlapped with a fruit on its whole path, as the AI won't find a good opportunity to slice it.

The game as a chrome extension: https://chrome.google.com/webstore/detail/fruit-ninja-game/fdkhnibpmdfmpgaipjiodbpdngccfibp

Simply place the chrome extension on the top right corner of your screen and run this file :) 

Some heuristics and timings might differ depending on your machine.
(too much/little computing time between frames might affect the AI's decisions)
'''


import time
from datetime import datetime
import cv2
import mss
import numpy as np
import os
import win32api, win32con
import pyautogui
import sys
import threading
from time import sleep
import math
import keyboard

DELAY_BETWEEN_SLICES = 0.19 # for sleep(DELAY_BETWEEN_SLICES)
DRAW_BOMBS = True
DEBUG = True
# pylint: disable=no-member,

screenHeight = win32api.GetSystemMetrics(1)
screenWidth = win32api.GetSystemMetrics(0)

'''
The game resolution is 750x500
'''
width = 750
height = 500

'''
I'm displaying my game at the top right corner of my screen
'''
gameScreen = {'top': 25, 'left': screenWidth - width, 'width': width, 'height': height}

today = datetime.now()

timeString = today.strftime('%b_%d_%Y__%H_%M_%S')

# Set writeVideo to True for saving screen captures for youtube
writeVideo = ( len(sys.argv) > 1 and sys.argv[1] == 'save' ) 

if(writeVideo):
    outputDir = './out/' + timeString
    os.mkdir(outputDir)
    fourcc1 = cv2.VideoWriter_fourcc(*'XVID')
    outScreen = cv2.VideoWriter(outputDir + '/screen.avi', fourcc1, 25, (width, height))

    fourcc2 = cv2.VideoWriter_fourcc(*'XVID')
    outResult = cv2.VideoWriter(outputDir + '/result.avi', fourcc2, 25, (width, height))

    fourcc3 = cv2.VideoWriter_fourcc(*'XVID')
    outMask = cv2.VideoWriter(outputDir + '/mask.avi', fourcc3, 25, (width, height))

def quit(): 
        
    if(writeVideo):
        outScreen.release()
        outResult.release()
        outMask.release()
    exit()


'''
Check if margins match screen coordinates
'''
def sanitizeMargins(rx, ry):
    margin = 10
    if (rx > width - margin):
        rx = width - margin
    if (rx < margin):
        rx = margin
    if ry > height - margin:
        ry = height - margin
    if ry < margin:
        ry = margin
    return (rx, ry)

'''
Translates from game screen coordinates to your monitor's screen coordinates
'''
def realCoord(x, y):
    rx = int(x)
    ry = int(y)
    rx, ry = sanitizeMargins(int(x), int(y))

    rx += gameScreen['left']
    ry += gameScreen['top']

    return rx, ry

def gameCoord(x, y):
    rx = int(x) - gameScreen['left']
    ry = int(y) - gameScreen['top']
    return sanitizeMargins(rx, ry)

'''
Moves mouse to (x,y) in screen coordinates
'''
def moveMouse(x, y):
    win32api.mouse_event(win32con.MOUSEEVENTF_MOVE | win32con.MOUSEEVENTF_ABSOLUTE, 
        int(x/screenWidth*65535.0), int(y/screenHeight*65535.0))

'''
Moves cursor from (x1,y1) to (x2, y2); 
'''
def swipe(_x1, _y1, _x2, _y2):
    x1, y1 = realCoord(_x1, _y1)
    x2, y2 = realCoord(_x2, _y2)
    
    # one line swipe is made of multiple cursor moves
    # if we instantly move the cursor too much, the game might not register the swipe
    points = 251

    for i in range(points+1):
        moveMouse((x1 * (points - i ) + x2 * i)/points, (y1 * (points -i ) + y2 * i)/points)
        if (i == 0):
            win32api.mouse_event(win32con.MOUSEEVENTF_LEFTDOWN,0, 0, 0, 0)
            moveMouse((x1 * (points -i ) + x2 * i)/points, (y1 * (points -i ) + y2 * i)/points)
    win32api.mouse_event(win32con.MOUSEEVENTF_LEFTUP,0, 0,0,0)
    moveMouse(x2, y2)
    time.sleep(DELAY_BETWEEN_SLICES)

    

swipeThread = None
cacheCoord = {} # keeps recently swiped points so we don't get stuck slicing the same apple
cacheDistance = 55 # distance between cached recent swipe points
cacheTime = 1 # seconds after which a cached point expires

def distPoints(x1, y1, x2, y2):
    return  math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )

'''
Checks if another thread is already swiping
'''
def canSwipe():
    return  (swipeThread is None ) or False == swipeThread.is_alive()

bombMinDistance = 58
bomdDown = 120

bomgHeightCond = height * 1/2

'''
Returns true if the line determined by [(x1, y1), (x2, y2)] doesn't slice a bomb or a bomb's path
'''
def lineIsSafe(x1S, y1S, x2S, y2S):
    global bombMinDistance
    for bomb in bombs:
        xBomb, yBomb = bomb

        minBombDistance = 999999999
        xPrev, yPrev = 0, 0
        
        # from the previous frame, find which bomb is closest to our current bomb, to predict the bomb's path
        # (closest bomb from the previous frame is probably the same bomb)
        for prevBomb in prevBombs:
            dist =  distPoints(prevBomb[0], prevBomb[1], xBomb, yBomb)
            if minBombDistance > dist:
                xPrev, yPrev = prevBomb
                dist = minBombDistance

        if xPrev == 0 and yPrev == 0:
            # no close bomb found (maybe the bomb just entered the scene?)
            continue
        
        # where do we expect the bomb to be, considering its previous position from the previous frame?
        predicts = [(xBomb + (xBomb - xPrev) * 3, yBomb + (yBomb - yPrev) * 3)]

        if (yBomb < bomgHeightCond):
            predicts.append((xBomb + (xBomb - xPrev) * 3 , yBomb + bomdDown))
            predicts.append((xBomb, yBomb + bomdDown))

        # check if our swiping line interferes with the bomb's path
        for predict in predicts:
            points = 10 # points to check among the swiped line
            for i in range(points+1):
                xInterm = (xBomb * (points - i ) + predict[0] * i)/points
                yInterm = (yBomb * (points -  i ) + predict[1] * i)/points
                
                points2 = 10 # points to check among the bomb's path
                for i in range(points2+1):
                    xPoint = (x1S * (points2 - i ) + x2S * i)/points2
                    yPoint = (y1S * (points2 -  i ) + y2S * i)/points2
                    dist = distPoints(xInterm, yInterm, xPoint, yPoint)
                    if (dist < bombMinDistance):
                        return False

        # check if our swiping lane interferes with the bomb's position
        points = 10
        for i in range(points+1):
            xPoint = (x1S * (points - i ) + x2S * i)/points
            yPoint = (y1S * (points -  i ) + y2S * i)/points
            dist = distPoints(xBomb, yBomb, xPoint, yPoint)
            if (dist < bombMinDistance):
                return False
    
    return True
        
def shouldSwipe(_x1, _y1):
    global swipeThread, img

    x1, y1 = realCoord(_x1, _y1)

    res = True
    for key in list(cacheCoord):
        x2, y2 = key
        # remove cached swipes which are too old
        if (cacheCoord[key] < time.time() - cacheTime):
            del cacheCoord[key]
            continue
        dist = distPoints(x1, y1, x2, y2)
        # is our current position too close to a cached area?

        if  dist < cacheDistance:
            res = False

    if res == False:
        return False
    

    # try different angles to swipe on fruit
    swipeTries = [
                (_x1, _y1 + 120, _x1, _y1 - 100),
                (_x1, _y1 + 80, _x1, _y1 - 50),
                (_x1 + 12, _y1 + 80, _x1 + 12, _y1 - 50),
                (_x1 - 12, _y1 + 80, _x1 - 12, _y1 - 50),
                (_x1 + 15, _y1 + 50, _x1 + 15, _y1 - 30),
                (_x1 - 15, _y1 + 50, _x1 - 15, _y1 - 30),
                ]

    for curTry in swipeTries:
        x1S, y1S, x2S, y2S = curTry
       
        # choose first line which doesn't cut a bomb
        # (usually the first line, hence why most slices are vertical)
        if lineIsSafe( x1S, y1S, x2S, y2S):
            #swipe in a separate thread
            swipeThread = threading.Thread(target=swipe, args=(x1S, y1S, x2S, y2S))
            swipeThread.start()
            
            points = 10

            # mark points along the line as recently swiped
            # so we don't slice in the same area continously
            for i in range(points+1):
                xPoint = (x1S * (points - i ) + x2S * i)/points
                yPoint = (y1S * (points -  i ) + y2S * i)/points
                cacheCoord[realCoord(xPoint, yPoint)] = time.time()
            return True

    return False

bombs = []
possibleSwipes = []

def orderFruitsBy(tup):
    # always slice the fruit closer to the edge
    x, y = tup
    toFilter = [height - y, x]
    if x > width * 4/5:
        toFilter.append(width - x)
    return min(toFilter)

with mss.mss() as sct:
    while True:
        last_time = time.time()
        screen = np.array(sct.grab(gameScreen))
        screen = np.flip(screen[:, :, :3], 2) 
        screen = cv2.cvtColor(screen, cv2.COLOR_BGR2RGB)
        img = screen.copy()

        # color ranges for object detections
        objectBoundaries = [([15, 180, 130], [35, 237, 209], 120, True), # fruit
                ([0, 40, 10], [30, 170, 50], 100, True), # fruit
                ([10, 50, 220], [40, 250, 255], 80, True), # fruit
                ([30, 30, 20], [60, 70, 60], 60, False), # bomb
                ]

        mask = None
        tmpMask = None

        prevSwipes = possibleSwipes
        possibleSwipes = []
        prevBombs = bombs
        bombs = []
        for boundary in objectBoundaries: 
            
            lower, upper, minPoints, isFruit = boundary
            tmpMask = cv2.inRange(img, np.array(lower), np.array(upper))

            contours, hierarchy = cv2.findContours(tmpMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if (mask is None):
                mask = tmpMask
            else:
                mask = cv2.bitwise_or(mask, tmpMask)

            for c in contours:
                if (len(c) < minPoints) :
                    # contour is too small, probably not a fruit/bomb
                    continue

                centerX, centerY, __1, __2 = map(int, cv2.mean(c))

                sumX, sumY = 0, 0
                rows = 70
                cols = 70
                cnt = 0

                # get center of current object
                for i in range(rows):
                    for j in range(cols):
                        x = int(centerX + i - rows/2)
                        y = int(centerY + j - cols/2)
                        if x < 0 or x >= width or y < 0 or y >= height:
                            continue
                        val = tmpMask[y, x]
                        if val:
                            sumX += x
                            sumY += y
                            cnt += 1
                if cnt == 0:
                    continue

                sumX = int(sumX/cnt)
                sumY = int(sumY/cnt)

                # pretty whie circle around the detected object
                cv2.circle(img, (sumX, sumY), 35, (255, 255, 255), 2)

                if isFruit:
                    # mark all swipable fruits
                    good = True
                    for swipePoint in possibleSwipes:
                        x, y = swipePoint
                        dist = distPoints(x, y, sumX, sumY)
                        if dist < 15:
                            good = False
                            break
                    yBarrier = int(height * 4/5)
                    if (sumY > yBarrier):
                        break
                    if good:
                        possibleSwipes.append((sumX, sumY))
                else:
                    bombs.append((sumX, sumY))

        # pretty circles for the video output :)
        if DRAW_BOMBS:
            for bomb in bombs:
                xBomb, yBomb = bomb
                minBombDistance = 999999999
                xPrev, yPrev = 0, 0
                for prevBomb in prevBombs:
                    dist =  distPoints(prevBomb[0], prevBomb[1], xBomb, yBomb)
                    if minBombDistance > dist:
                        xPrev, yPrev = prevBomb
                        dist = minBombDistance
                if xPrev == 0 and yPrev == 0:
                    continue
                predicts = [(xBomb + (xBomb - xPrev) * 3, yBomb + (yBomb - yPrev) * 3)]
                for predict in predicts:
                    cv2.circle(img, predict, bombMinDistance, (255, 0, 0), 10)
                    cv2.circle(img, bomb, bombMinDistance, (255, 0, 0), 10)

        debug = img.copy()
        possibleSwipes.sort(key=lambda tup: orderFruitsBy(tup), )
        if canSwipe():
            for elem in possibleSwipes:
                centerX, centerY = elem
                if (shouldSwipe(centerX, centerY)):
                    break

        if canSwipe():
            # still nothing swiped? probably because every area was 
            # already cached, so we remove the cache and retry !
            cacheCoord = {}
            for elem in possibleSwipes:
                centerX, centerY = elem
                if (shouldSwipe(centerX, centerY)):
                    break

        if DEBUG:
            for coords in cacheCoord:
                cv2.circle(debug, gameCoord(coords[0], coords[1]), cacheDistance, (123, 123, 0), 1)

        #cv2.imshow('result', tmpMask)
        cv2.imshow('debug', debug)
        cv2.imshow('img', img)
        
        if writeVideo:
            outResult.write(img)
            outScreen.write(screen)            
            outMask.write(debug)


        cv2.waitKey(1)
        # Press 'q' to quit
        if keyboard.is_pressed('q'):
            cv2.destroyAllWindows()
            quit()
quit()
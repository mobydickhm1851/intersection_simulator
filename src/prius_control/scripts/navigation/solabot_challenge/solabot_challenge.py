#!/usr/bin/env python

import pygame
from pygame.locals import *


WINDOWWIDTH = 640
WINDOWHEIGHT = 480
CELLSIZE = 20

#             R    G    B
WHITE     = (255, 255, 255)
GREY      = (222, 222, 222)
BLACK     = (  0,   0,   0)
SOLABLUE  = ( 73, 159, 227)
PINK      = ( 227, 73, 129)
DARKGRAY  = ( 80,  80,  80)
BGCOLOR = BLACK

pygame.init()
DISPLAYSURF = pygame.display.set_mode((WINDOWWIDTH,WINDOWHEIGHT))    
BASICFONT = pygame.font.Font('freesansbold.ttf', 18)

def terminate():
    pygame.quit()
    sys.exit()

# This function terminate if enent QUIT is triggered or 'esc' is pressed. Return none if no key is pressed, if otherwise, return the pressed key.
def checkForKeyPress():
    if (pygame.event.get(QUIT)):
        terminate()

    keyUpEvents = pygame.event.get(KEYUP)
    if len(keyUpEvents) == 0:
        return None
    if keyUpEvents[0].key == K_ESCAPE:
        terminate()
    return keyUpEvents[0].key

def drawGrid():
    for x in range(0, WINDOWWIDTH, CELLSIZE): # draw vertical lines
        pygame.draw.line(DISPLAYSURF, DARKGRAY, (x, 0), (x, WINDOWHEIGHT))
    for y in range(0, WINDOWHEIGHT, CELLSIZE): # draw horizontal lines
        pygame.draw.line(DISPLAYSURF, DARKGRAY, (0, y), (WINDOWWIDTH, y))


def showStartScreen():
    DISPLAYSURF.fill(BGCOLOR)
    titleFont = pygame.font.Font('freesansbold.ttf', 88)
    title1 = titleFont.render('SOLabot', True, GREY)
    title2 = titleFont.render('Challenge ', True, GREY)
    Rect1 = title1.get_rect()
    Rect2 = title2.get_rect()
    Rect1.midtop = (WINDOWWIDTH / 2, 115)
    Rect2.midtop = (WINDOWWIDTH / 2, Rect1.height + 135)

    DISPLAYSURF.blit(title1, Rect1)
    DISPLAYSURF.blit(title2, Rect2)

    drawPressKeyMsg()    
    pygame.display.update()
    checkForKeyPress() # clear out keey pressed in event queue

    while True:
        if checkForKeyPress():
            pygame.event.get() # clear event queue
            return


def showTutorial():
    DISPLAYSURF.fill(BGCOLOR)
    # title
    titleFont = pygame.font.Font('freesansbold.ttf', 44)
    title1 = titleFont.render('SOLabot', True, GREY)
    title2 = titleFont.render('Challenge ', True, GREY)
    Rect1 = title1.get_rect()
    Rect2 = title2.get_rect()
    Rect1.midtop = (WINDOWWIDTH / 2, 10)
    Rect2.midtop = (WINDOWWIDTH / 2, Rect1.height + 10)

    DISPLAYSURF.blit(title1, Rect1)
    DISPLAYSURF.blit(title2, Rect2)

    # intro
    titleFont = pygame.font.Font('freesansbold.ttf', 16)
    text1 = "Hi! SOLabors! Welcome to the SOLabot Challenge!"
    text2 = "We are looking for a team of two elites to join our program"
    text3 = 'but first you have to accept this challenge!'
    text4 = "The task is easy: Each member controls one SOLabot and "
    text5 = "try to traverse the intersection without collision as fast as possible." 
    text6 = "Sounds easy right! But remember, the timer will stop "
    text7 = "only when BOTH of members arrive at their goals(traverse the intersection)."
    text8 = "Good luck! Roll out when ready!"

    title1 = titleFont.render(text1, True, GREY)
    title2 = titleFont.render(text2, True, GREY)
    title3 = titleFont.render(text3, True, GREY)
    title4 = titleFont.render(text4, True, GREY)
    title5 = titleFont.render(text5, True, GREY)
    title6 = titleFont.render(text6, True, GREY)
    title7 = titleFont.render(text7, True, GREY)
    title8 = titleFont.render(text8, True, GREY)
    Rect1 = title1.get_rect()
    Rect2 = title2.get_rect()
    Rect3 = title3.get_rect()
    Rect4 = title4.get_rect()
    Rect5 = title5.get_rect()
    Rect6 = title6.get_rect()
    Rect7 = title7.get_rect()
    Rect8 = title8.get_rect()
    Rect1.midtop = (WINDOWWIDTH / 2, 150)
    Rect2.midtop = (WINDOWWIDTH / 2, 170)
    Rect3.midtop = (WINDOWWIDTH / 2, 190)
    Rect4.midtop = (WINDOWWIDTH / 2, 210)
    Rect5.midtop = (WINDOWWIDTH / 2, 230)
    Rect6.midtop = (WINDOWWIDTH / 2, 250)
    Rect7.midtop = (WINDOWWIDTH / 2, 270)
    Rect8.midtop = (WINDOWWIDTH / 2, 290)

    DISPLAYSURF.blit(title1, Rect1)
    DISPLAYSURF.blit(title2, Rect2)
    DISPLAYSURF.blit(title3, Rect3)
    DISPLAYSURF.blit(title4, Rect4)
    DISPLAYSURF.blit(title5, Rect5)
    DISPLAYSURF.blit(title6, Rect6)
    DISPLAYSURF.blit(title7, Rect7)
    DISPLAYSURF.blit(title8, Rect8)
    

    # player 1
    intro1 = BASICFONT.render('PLAYER 1', True, SOLABLUE)
    intro2 = BASICFONT.render('W : move forward', True, WHITE)
    intro3 = BASICFONT.render('S : brake', True, WHITE)
    intro4 = BASICFONT.render('X : move backward', True, WHITE)
    intro1Rect = intro1.get_rect()
    intro2Rect = intro2.get_rect()
    intro3Rect = intro3.get_rect()
    intro4Rect = intro4.get_rect()
    intro1Rect.midtop = (175,350)
    intro2Rect.midtop = (175,375)
    intro3Rect.midtop = (175+(intro3Rect.width-intro2Rect.width)/2,400)
    intro4Rect.midtop = (175+(intro4Rect.width-intro2Rect.width)/2,425)
    # player 2
    intro5 = BASICFONT.render('PLAYER 2', True, PINK)
    intro6 = BASICFONT.render('O : move forward', True, WHITE)
    intro7 = BASICFONT.render('L : brake', True, WHITE)
    intro8 = BASICFONT.render('. : move backward', True, WHITE)
    intro5Rect = intro5.get_rect()
    intro6Rect = intro6.get_rect()
    intro7Rect = intro7.get_rect()
    intro8Rect = intro8.get_rect()
    intro5Rect.midtop = (445,350)
    intro6Rect.midtop = (445,375)
    intro7Rect.midtop = (445+(intro7Rect.width-intro6Rect.width)/2,400)
    intro8Rect.midtop = (445+(intro8Rect.width-intro6Rect.width)/2,425)

    DISPLAYSURF.blit(intro1, intro1Rect)
    DISPLAYSURF.blit(intro2, intro2Rect)
    DISPLAYSURF.blit(intro3, intro3Rect)
    DISPLAYSURF.blit(intro4, intro4Rect)
    DISPLAYSURF.blit(intro5, intro5Rect)
    DISPLAYSURF.blit(intro6, intro6Rect)
    DISPLAYSURF.blit(intro7, intro7Rect)
    DISPLAYSURF.blit(intro8, intro8Rect)

    pygame.display.update()
    checkForKeyPress() # clear out keey pressed in event queue

    while True:
        if checkForKeyPress():
            pygame.event.get() # clear event queue
            return

def countDown():
    TICKS = pygame.time.get_ticks()  
    countFrom = 3
    countFont = pygame.font.Font('freesansbold.ttf', 88)
    while True:

        seconds=(pygame.time.get_ticks()-TICKS)/1000 #calculate how many seconds

        if seconds > countFrom: # if more than 10 seconds close the game
            return
        if int(seconds) == countFrom:
            text = 'GO !!!'
        else:
            text = str(countFrom - seconds) #print how many seconds

        DISPLAYSURF.fill(BGCOLOR)
        countSurf = countFont.render(text, True, WHITE)
        countRect = countSurf.get_rect()
        countRect.center = (320, 240)
        countBlit = DISPLAYSURF.blit(countSurf, countRect)
        pygame.display.update()



def drawPressKeyMsg():
    pressKeySurf = BASICFONT.render('Press a key to play.', True, DARKGRAY)
    pressKeyRect = pressKeySurf.get_rect()
    pressKeyRect.topleft = (WINDOWWIDTH - 400, WINDOWHEIGHT - 100)
    DISPLAYSURF.blit(pressKeySurf, pressKeyRect)


def main():
    
    pygame.display.set_caption("SOLabot Challenge!!")
    drawGrid()
    showStartScreen()
    showTutorial()
    

    while True:
        for event in pygame.event.get() :
            if event.type == pygame.KEYDOWN :
                if event.key == pygame.K_SPACE :
                    print "Space bar pressed down."
                elif event.key == pygame.K_ESCAPE :
                    print "Escape key pressed down."
            elif event.type == pygame.KEYUP :
                if event.key == pygame.K_SPACE :
                    print "Space bar released."
                elif event.key == pygame.K_ESCAPE :
                    print "Escape key released."
            elif event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        pygame.display.update


if __name__ == '__main__':
    main()

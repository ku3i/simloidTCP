/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

// main window and event handling for X11

//#include <ode/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/keysym.h>
#include <GL/glx.h>

#include <draw/drawstuff.h>
#include <draw/internal.h>
#include <draw/version.h>
#include <basic/version.h>

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

//***************************************************************************
// error handling for unix

static void printMessage (const char *msg1, const char *msg2, va_list ap)
{
  fflush (stderr);
  fflush (stdout);
  fprintf (stderr,"\n%s: ",msg1);
  vfprintf (stderr,msg2,ap);
  fprintf (stderr,"\n");
  fflush (stderr);
}

extern "C" void dsError (const char *msg, ...)
{
  va_list ap;
  va_start(ap, msg);
  printMessage ("Error: ", msg, ap);
  exit(EXIT_FAILURE);
}

extern "C" void dsDebug (const char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  printMessage ("INTERNAL ERROR",msg,ap);
  abort();
}

extern "C" void dsPrint (const char *msg, ...)
{
  va_list ap;
  va_start(ap,msg);
  printf("%s›››%s", KYEL, KNRM);
  vprintf(msg,ap);
}

//***************************************************************************
// openGL window

// X11 display info
static Display *display=0;
static int screen=0;
static XVisualInfo *visual=0;		// best visual for openGL
static Colormap colormap=0;		// window's colormap
static Atom wm_protocols_atom = 0;
static Atom wm_delete_window_atom = 0;

// window and openGL
static Window win=0;			// X11 window, 0 if not initialized
static int width=0,height=0;		// window size
static GLXContext glx_context=0;	// openGL rendering context
static int last_key_pressed=0;		// last key pressed in the window
static int run=1;			// 1 if simulation running
static int pause=0;			// 1 if in `pause' mode
static int singlestep=0;		// 1 if single step key pressed
static int writeframes=0;		// 1 if frame files to be written
static int slowMotion=1;		// 1 if every frame to be written, every 4th otherwise


static void createMainWindow (int _width, int _height)
{
  // create X11 display connection
  display = XOpenDisplay (NULL);
  if (!display) dsError ("can not open X11 display");
  screen = DefaultScreen(display);

  // get GL visual
  static int attribList[] = {GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE,16,
			     GLX_RED_SIZE,4, GLX_GREEN_SIZE,4,
			     GLX_BLUE_SIZE,4, None};
  visual = glXChooseVisual (display,screen,attribList);
  if (!visual) dsError ("no good X11 visual found for OpenGL");

  // create colormap
  colormap = XCreateColormap (display,RootWindow(display,screen),
			      visual->visual,AllocNone);

  // initialize variables
  win = 0;
  width = _width;
  height = _height;
  glx_context = 0;
  last_key_pressed = 0;

  if (width < 1 || height < 1) dsDebug (0,"bad window width or height");

  // create the window
  XSetWindowAttributes attributes;
  attributes.background_pixel = BlackPixel(display,screen);
  attributes.colormap = colormap;
  attributes.event_mask = ButtonPressMask | ButtonReleaseMask |
    KeyPressMask | KeyReleaseMask | ButtonMotionMask | PointerMotionHintMask |
    StructureNotifyMask;
  win = XCreateWindow (display,RootWindow(display,screen),50,50,width,height,
		       0,visual->depth, InputOutput,visual->visual,
		       CWBackPixel | CWColormap | CWEventMask,&attributes);

  // associate a GLX context with the window
  glx_context = glXCreateContext (display,visual,0,GL_TRUE);
  if (!glx_context) dsError ("can't make an OpenGL context");

  // set the window title
  XTextProperty window_name;
  window_name.value = (unsigned char *) version::name.c_str();
  window_name.encoding = XA_STRING;
  window_name.format = 8;
  window_name.nitems = strlen((char *) window_name.value);
  XSetWMName (display,win,&window_name);

  // participate in the window manager 'delete yourself' protocol
  wm_protocols_atom = XInternAtom (display,"WM_PROTOCOLS",False);
  wm_delete_window_atom = XInternAtom (display,"WM_DELETE_WINDOW",False);
  if (XSetWMProtocols (display,win,&wm_delete_window_atom,1)==0)
    dsError ("XSetWMProtocols() call failed");

  // pop up the window
  XMapWindow (display,win);
  XSync (display,win);
}


static void destroyMainWindow()
{
  glXDestroyContext (display,glx_context);
  XDestroyWindow (display,win);
  XSync (display,0);
  display = 0;
  win = 0;
  glx_context = 0;
}


static void handleEvent (XEvent &event, dsFunctions *fn)
{
  static int mx=0,my=0; 	// mouse position
  static int mode = 0;		// mouse button bits

  switch (event.type) {

  case ButtonPress: {
    if (event.xbutton.button == Button1) mode |= 1;
    if (event.xbutton.button == Button2) mode |= 2;
    if (event.xbutton.button == Button3) mode |= 4;
    mx = event.xbutton.x;
    my = event.xbutton.y;
  }
  return;

  case ButtonRelease: {
    if (event.xbutton.button == Button1) mode &= (~1);
    if (event.xbutton.button == Button2) mode &= (~2);
    if (event.xbutton.button == Button3) mode &= (~4);
    if (event.xbutton.button == Button4) { dsWheelMotion(1); }          //Wheel up
    if (event.xbutton.button == Button5) { dsWheelMotion(-1); }		//Wheel down
    mx = event.xbutton.x;
    my = event.xbutton.x;
  }
  return;

  case MotionNotify: {
    if (event.xmotion.is_hint) {
      Window root,child;
      unsigned int mask;
      XQueryPointer (display,win,&root,&child,&event.xbutton.x_root,
		     &event.xbutton.y_root,&event.xbutton.x,&event.xbutton.y,
		     &mask);
    }
    dsMotion (mode, event.xmotion.x - mx, event.xmotion.y - my);
    mx = event.xmotion.x;
    my = event.xmotion.y;
  }
  return;

  case KeyPress: {
    KeySym key;
    XLookupString (&event.xkey,NULL,0,&key,0);

	//alles ohne Strg-Taste: weiterreichen an user-defined command function
    if ((event.xkey.state & ControlMask) == 0) {
      if (key >= ' ' && key <= 126 && fn->command) fn->command (key, (event.xkey.state & ShiftMask));
    }
    else if (event.xkey.state & ControlMask) {
      switch (key) {
		case 't': case 'T':
	dsSetTextures (dsGetTextures() ^ 1);
	break;
      case 's': case 'S':
	dsSetShadows (dsGetShadows() ^ 1);
	break;
      case 'x': case 'X':
	run = 0;
	break;
      case 'p': case 'P':
	pause ^= 1;
	singlestep = 0;
	break;
      case 'o': case 'O':
	if (pause) singlestep = 1;
	break;
      case 'v': case 'V': {
	float xyz[3],hpr[3];
	dsGetViewpoint (xyz,hpr);
	printf ("Viewpoint = (%.4f,%.4f,%.4f,%.4f,%.4f,%.4f)\n",
		xyz[0],xyz[1],xyz[2],hpr[0],hpr[1],hpr[2]);
	break;
      }
	case 'w': case 'W':
		writeframes ^= 1;
		if (writeframes) printf ("Now writing frames to PPM files\n");
		break;
	case 'j': case 'J':
		slowMotion ^= 1;
		if (slowMotion) printf ("Drawing every frame\n");
		else printf ("Drawing only every 4th frame\n");
		break;


      }
    }
    last_key_pressed = key;		// a kludgy place to put this...
  }
  return;

  case KeyRelease: {
    // hmmmm...
  }
  return;

  case ClientMessage:
    if (event.xclient.message_type == wm_protocols_atom &&
	event.xclient.format == 32 &&
	Atom(event.xclient.data.l[0]) == wm_delete_window_atom) {
      run = 0;
      return;
    }
    return;

  case ConfigureNotify:
    width = event.xconfigure.width;
    height = event.xconfigure.height;
    return;
  }
}


// return the index of the highest bit
static int getHighBitIndex (unsigned int x)
{
  int i = 0;
  while (x) {
    i++;
    x >>= 1;
  }
  return i-1;
}


// shift x left by i, where i can be positive or negative
#define SHIFTL(x,i) (((i) >= 0) ? ((x) << (i)) : ((x) >> (-i)))


static void captureFrame(int num)
{
    fprintf(stderr, "capturing frame %04d\n", num);

    char s[32];
    snprintf(s, 32, "frame%04d.ppm", num);
    FILE *f = fopen(s, "wb");

    if (!f) dsError("can't open \"%s\" for writing", s);
    fprintf(f, "P6\n%d %d\n255\n", width, height);

    XImage *image = XGetImage(display, win, 0, 0, width, height, ~0, ZPixmap);

    int rshift = 7 - getHighBitIndex(image->  red_mask);
    int gshift = 7 - getHighBitIndex(image->green_mask);
    int bshift = 7 - getHighBitIndex(image-> blue_mask);

    for (int y=0; y<height; y++) {
        for (int x=0; x<width; x++) {
            unsigned long pixel = XGetPixel (image,x,y);
            unsigned char b[3];
            b[0] = SHIFTL(pixel & image->  red_mask, rshift);
            b[1] = SHIFTL(pixel & image->green_mask, gshift);
            b[2] = SHIFTL(pixel & image-> blue_mask, bshift);
            fwrite(b, 3, 1, f);
        }
    }
    fclose(f);
    XDestroyImage(image);
}


void dsPlatformSimLoop (int window_width, int window_height, dsFunctions *fn, int initial_pause, int record_frames)
{
    pause = initial_pause;
    writeframes = record_frames;

    if(!fn->disableGraphics)
    {
        createMainWindow(window_width, window_height);
        glXMakeCurrent(display, win, glx_context);
        dsStartGraphics(window_width, window_height, fn);

    if (!fn->disableGraphics)
        fprintf( stderr
               , "Simulation controls: (dsDrawstuff v%d.%02d):\n"
                 "   Ctrl-P - pause / unpause simulation (or say `--pause' on command line)\n"
                 "   Ctrl-O - single step when paused\n"
                 "   Ctrl-X - exit\n"
                 "\n"
                 "Graphic controls:\n"
                 "   Ctrl-T - toggle textures (or say `--notex' on command line)\n"
                 "   Ctrl-S - toggle shadows (or say `--noshadow' on command line)\n"
                 "   Ctrl-V - print current viewpoint coordinates (x,y,z,h,p,r)\n"
                 "   Ctrl-W - write frames to ppm files: frame/frameNNN.ppm\n"
                 "   Ctrl-J - write frames in Slow Motion 100Hz\n"
                 "\n"
                 "Camera controls:\n"
                 "   A/D           - left / right\n"
                 "   W/S           - forward / backward\n"
                 "   Left button   - pan and tilt\n"
                 "   Right button  - forward and sideways\n"
                 "   Middle button - sideways and up (also: left+right button)\n"
                 "   Mouse wheel   - up and down\n"
                 "   T             - toggle automatically rotating camera on/off\n"
                 "\n"
               , DS_VERSION >> 8
               , DS_VERSION & 0xff
               );
    } else
      fprintf(stderr, "Graphics disabled.\n\n");

    if (fn->start) fn->start();

    int frame = 1;
    run = 1;
    while (run && *(fn->continueLoop))
    {
        /* read in and process all pending events for the main window */
        if (!fn->disableGraphics)
        {
            XEvent event;
            while (run && XPending(display))
            {
                XNextEvent(display, &event);
                handleEvent(event, fn);
            }
        }

        dsDrawFrame(width, height, fn, pause && !singlestep, singlestep);
        singlestep = 0;

        if(*(fn->drawScene))
        {
            glFlush();
            glXSwapBuffers (display,win);
            XSync (display,0);

            // capture frames if necessary
            if (pause==0 && (writeframes || *(fn->recordFrames)))
            {
                if (!slowMotion)
                {
                    if (frame%4 == 0)
                        captureFrame(frame); //25fps at dt = 0.01 s
                }
                else
                    captureFrame(frame);

                frame++;
            }
        }
    }

    if (fn->stop)
        fn->stop();

    if(!fn->disableGraphics)
    {
        dsStopGraphics();
        destroyMainWindow();
    }
}


extern "C" void dsStop()
{
  run = 0;
}

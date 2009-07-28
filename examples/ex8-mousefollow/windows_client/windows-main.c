/* mouse_follow                                                             *
 *                                                                          *
 * Author: Christopher Dellin                                               *
 *   Date: 2009-01-26                                                       * 
 *                                                                          *
 * Description.                                                             *
 *                                                                          */ 

/* Standard Windows header file */
#include <windows.h>
#include <winsock2.h>
#include <stdio.h> /* For sprintf */

#include "windows-resources.h"

#define ID_BUTTON_SETIP 1

#define ID_LISTBOX 2
#define ID_BUTTON_CONNECT 3

SOCKET sock;
struct sockaddr_in their_addr;
int their_addr_good = 0;

/* Window Callback */
LRESULT CALLBACK WndProc(HWND hwnd, UINT iMsg, WPARAM wParam, LPARAM lParam);

HWND w; /* Window Instance Handle*/
HWND w_poslabel;
HWND w_ip;
HWND w_setip;
/*
 HWND w_button_find;
HWND w_listbox;
HWND w_button_connect;
 */

/*  Program Entry Point */
int WINAPI WinMain(HINSTANCE hInstance,     /* Handle to current instance */
                   HINSTANCE hPrevInstance, /* Handle to previous instance */
		             LPSTR szCmdLine,         /* Command line arguments */
                   int iCmdShow)            /* Show state of window
                                               (max, min, etc) */
{
   int err;
   WSADATA wsaData;
   WNDCLASSEX w_class;  /* Window Class */
   RECT rcl;
   MSG msg;
   
   /* Start WinSock, create UDP socket */
   WSAStartup(MAKEWORD(2,2), &wsaData);
   sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
   if (sock == INVALID_SOCKET)
      return -1;
   
   /* Fill in Window Class */
   ZeroMemory(&w_class,sizeof(w_class));
   w_class.cbSize        = sizeof(w_class);
   w_class.style         = CS_HREDRAW | CS_VREDRAW;
   w_class.lpfnWndProc   = WndProc; /* Window Callback */
   w_class.cbClsExtra    = 0;
   w_class.cbWndExtra    = 0;
   w_class.hInstance     = hInstance; /* Make this window in my instance. */
   w_class.hIcon         = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_LOGO32));
   w_class.hIconSm       = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_LOGO32));
   w_class.hCursor       = LoadCursor(NULL, IDC_ARROW);
   /*w_class.hbrBackground = (HBRUSH) GetStockObject(WHITE_BRUSH);*/
   w_class.hbrBackground = (HBRUSH)(COLOR_BACKGROUND);
   w_class.lpszClassName = "WAMMouseFollowWindowClass";
   w_class.lpszMenuName  = NULL;

   /* Register this Window Class with Windows  */
   RegisterClassEx(&w_class);

   /*  Create a window based on our new class  */
   w = CreateWindow("WAMMouseFollowWindowClass",
                    "WAM Mouse Follow Example", /* Window Title */
                    WS_CAPTION | WS_BORDER | WS_SYSMENU, /* Window Style */
                    CW_USEDEFAULT, CW_USEDEFAULT, /* (x,y) position */
                    /*CW_USEDEFAULT, CW_USEDEFAULT,*/ /* width, height */
                    630, 470, /* width, height */
                    NULL, /* Parent */
                    NULL, hInstance, NULL);
   
   /* Create a position label */
   w_poslabel = CreateWindow(TEXT("EDIT"),
                          TEXT("Mouse Position"),
                          WS_CHILD | WS_VISIBLE |
                             ES_LEFT | ES_READONLY,
                          5, 5,
                          140, 25,
                          w,
                          NULL, hInstance, NULL);

   w_ip = CreateWindow(TEXT("EDIT"),
                          TEXT("IP Address"),
                          WS_CHILD | WS_VISIBLE | ES_LEFT,
                          150, 5,
                          130, 25,
                          w,
                          NULL, hInstance, NULL);
   
   w_setip = CreateWindow(TEXT("BUTTON"),
                               TEXT("Set IP"),
                               WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                               290, 5,
                               80, 25,
                               w,
                               (HMENU)ID_BUTTON_SETIP,
                               hInstance, NULL);
   
#if 0
   /* Create a find button */
   w_button_find = CreateWindow(TEXT("BUTTON"),
                               TEXT("Refresh List"),
                               WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                               5+200+5, 5,
                               150, 25,
                               w,
                               (HMENU)ID_BUTTON_FIND,
                               hInstance, NULL);
   
   /* Create a listbox */
   w_listbox = CreateWindow(TEXT("LISTBOX"),
                            TEXT("Listbox Test"),
                            WS_CHILD | WS_VISIBLE | LBS_STANDARD | LBS_DISABLENOSCROLL | LBS_NOINTEGRALHEIGHT,
                            0, 35,
                            rcl.right - rcl.left, 100,
                            w,
                            (HMENU)ID_LISTBOX,
                            hInstance, NULL);
   SendMessage( w_listbox, LB_SETITEMHEIGHT, 0, 30 );
   
   /* Create a connect button */
   w_button_connect = CreateWindow(TEXT("button"),
                                   TEXT("Connect"),
                                   WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON,
                                   5+200+5, 150,
                                   150, 25,
                                   w,
                                   (HMENU)ID_BUTTON_CONNECT,
                                   hInstance, NULL);
#endif
   /*  Show and update our window  */
   ShowWindow(w, iCmdShow);
   UpdateWindow(w);
   
   /* Forward socket events to the message-based notification system */
   /*WSAAsyncSelect( get_socket(), w, WM_USER+1, FD_READ );*/

   /*  Retrieve and process messages until we get WM_QUIT  */
   while ( GetMessage(&msg, NULL, 0, 0) )
   {
      TranslateMessage(&msg);    /*  for certain keyboard messages  */
      DispatchMessage(&msg);     /*  send message to WndProc        */
   } 
   
   closesocket(sock);
   WSACleanup();
   
   /*  Exit with status specified in WM_QUIT message  */
   return msg.wParam;
}

/* Window Callback */
LRESULT CALLBACK WndProc(HWND hwnd, UINT iMsg, WPARAM wParam, LPARAM lParam)
{
   PAINTSTRUCT ps;
   HDC         hdc;
   
   /*  Switch according to what type of message we have received  */
   switch ( iMsg )
   {
      case WM_PAINT:
      {
         PAINTSTRUCT ps;
         HDC hdc;
         hdc = BeginPaint( w, &ps );
         
         Rectangle(hdc,10,35,610,435);
         
         EndPaint(w, &ps);
         return 0;
      }
      case WM_MOUSEMOVE:
      {
         POINT pt;
         char string[100];
         char buffer[10];
         int x, y;
         
         GetCursorPos(&pt);
         ScreenToClient(w, &pt);
         x = pt.x - 10;
         y = pt.y - 35;
         
         if (x < 0) x = 0;
         if (y < 0) y = 0;
         
         if (x > 600) x = 600;
         if (y > 400) y = 400;
      
         sprintf(buffer,"%04d|%04d",x,y);
         sprintf(string,"pos: %s",buffer);
         
         SendMessage(w_poslabel, WM_SETTEXT, 0, (LPARAM) string);
         
         if (their_addr_good)
         {
            sendto(sock, buffer, 10, 0, (struct sockaddr *)&their_addr, sizeof(struct sockaddr));
         }
         return 0;
      }
      case WM_CREATE:
      {
         /*
         if (send_broadcast() == -1)
            MessageBox(w,"Could not send broadcast packet. Check local firewall settings.","Error",MB_OK|MB_ICONERROR);
          */
         return 0;
      }
      case WM_SIZE:
      {
         RECT rcl;
         int fw; /* full-width */
         int hw; /* half-width */
         int right_offset; /* right-column offset */
         int lb_height;
         int bottom_offset;
         
         GetClientRect(w, &rcl);
#if 0
         /* Calculate horizontals */
         fw = (rcl.right-rcl.left) - 10;
         hw = ((rcl.right-rcl.left) - 15) / 2;
         right_offset = 10 + hw;
         
         /* Calculate verticals */
         lb_height = (rcl.bottom-rcl.top) - 40 - 40;
         bottom_offset = 40 + lb_height + 5;
         
         MoveWindow(w_label,                  5, 10, hw,  20, TRUE);
         MoveWindow(w_button_find, right_offset,  5, hw,  30, TRUE);
         
         MoveWindow(w_listbox,                5, 40, fw, lb_height, TRUE);
         
         MoveWindow(w_button_connect, right_offset, bottom_offset, hw, 30, TRUE );
#endif
         return 0;
      }
#if 0
      case WM_USER+1:
      {
         int i;
         int listbox_i;
         int num;
         char addbuf[100];
         
         i = receive_packet();
         if (i < 0)
         {
            MessageBox(w,"Error receiving packet","Error",MB_OK|MB_ICONERROR);
            return 0;
         }
         
         /* Put the entry in the list box, with its index as item data */
         num = list_get_num(i);
         if ( num >= 0 )
            sprintf(addbuf,"WAM %d (%s)", num, list_get_ipstr(i));
         else
            sprintf(addbuf,"WAM ?? (%s)", list_get_ipstr(i));
         listbox_i = (int) SendMessage(w_listbox, LB_ADDSTRING, 0, (LPARAM) TEXT(addbuf) );
         SendMessage(w_listbox, LB_SETITEMDATA, listbox_i, (LPARAM)i);
         
         if (i == 1)
            SendMessage(w_listbox,LB_SETSEL,TRUE,0);
         
         return 0;
      }
#endif
      case WM_COMMAND:
         switch (LOWORD(wParam))
         {
            case ID_BUTTON_SETIP:
            {
               char buf[100];
               struct sockaddr_in addr;
               
               GetWindowText(w_ip,buf,100);
               
               addr.sin_family = AF_INET;
               addr.sin_port = htons(1338);
               addr.sin_addr.s_addr = inet_addr(buf);
               
               if (addr.sin_addr.s_addr != INADDR_NONE)
               {
                  their_addr = addr;
                  their_addr_good = 1;
               }
               else
               {
                  MessageBox(w,"Bad IP Address!","Error",MB_OK|MB_ICONERROR);
                  their_addr_good = 0;
               }
               
               return 0;
            }
#if 0
            case ID_BUTTON_CONNECT:
               call_putty();
               return 0;
            case ID_LISTBOX:
            {
               switch (HIWORD(wParam))
               {
                  case LBN_DBLCLK:
                     call_putty();
                     return 0;
               }

               return 0;
            }
#endif
         }
         return 0;
            

      case WM_DESTROY:
         /* Window has been destroyed, so exit cleanly */
         PostQuitMessage(0);
	      return 0;
   }
   
   /*  Send any messages we don't handle to default window procedure  */
   return DefWindowProc(hwnd, iMsg, wParam, lParam);
}



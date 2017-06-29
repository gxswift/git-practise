/*********************************************************************
*                                                                    *
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                                                                    *
**********************************************************************
*                                                                    *
* C-file generated by:                                               *
*                                                                    *
*        GUI_Builder for emWin version 5.32                          *
*        Compiled Oct  8 2015, 11:59:02                              *
*        (c) 2015 Segger Microcontroller GmbH & Co. KG               *
*                                                                    *
**********************************************************************
*                                                                    *
*        Internet: www.segger.com  Support: support@segger.com       *
*                                                                    *
**********************************************************************
*/

// USER START (Optionally insert additional includes)
// USER END

#include "DIALOG.h"

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define ID_WINDOW_0 (GUI_ID_USER + 0x00)
#define ID_BUTTON_0 (GUI_ID_USER + 0x02)
#define ID_BUTTON_1 (GUI_ID_USER + 0x03)
#define ID_LISTWHEEL_0 (GUI_ID_USER + 0x04)

#define DATE_DIS (WM_USER+1)
// USER START (Optionally insert additional defines)
// USER END

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

// USER START (Optionally insert additional static data)
// USER END

/*********************************************************************
*
*       _aDialogCreate
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { WINDOW_CreateIndirect, "Window", ID_WINDOW_0, 0, 50, 800, 430, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "return", ID_BUTTON_0, 715, 346, 60, 45, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "OK", ID_BUTTON_1, 615, 346, 60, 45, 0, 0x0, 0 },
  { LISTWHEEL_CreateIndirect, "Listwheel", ID_LISTWHEEL_0, 644, 30, 130, 298, 0, 0x0, 0 },
  // USER START (Optionally insert additional widgets)
  // USER END
};

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/
static int _OwnerDraw(const WIDGET_ITEM_DRAW_INFO * pDrawItemInfo) {
	switch (pDrawItemInfo->Cmd) {
	case WIDGET_DRAW_OVERLAY:
		GUI_SetColor(GUI_RED);
		GUI_DrawHLine(125, 0, 130);
		GUI_DrawHLine(126, 0, 130);
		GUI_DrawHLine(127, 0, 130);
		GUI_DrawHLine(179, 0, 130);
		GUI_DrawHLine(180, 0, 130);
		GUI_DrawHLine(181, 0, 130);
	/*	break;
	case WIDGET_DRAW_BACKGROUND:*/
		GUI_DrawGradientV(0, 0, 130, 125, GUI_DARKBLUE, 0xFF000000 | GUI_BLUE);
		GUI_DrawGradientV(0, 180, 130, 298, 0xFF000000 | GUI_BLUE,  GUI_DARKBLUE);
		break;
	default:
		return LISTWHEEL_OwnerDraw(pDrawItemInfo);
	}
	return 0;
}
// USER START (Optionally insert additional static code)
// USER END
extern WM_HWIN CreateWin1(void);
extern WM_HWIN WIN_Header;
CALENDAR_DATE Date;

	int Hour;
	int Min;
	int Sec;

/*********************************************************************
*
*       _cbDialog
*/
static void _cbDialog(WM_MESSAGE * pMsg) {
  WM_HWIN hItem;
  int     NCode;
  int     Id;
  static WM_HWIN hCalen;

  static int Select;
  static int Select2;
  // USER START (Optionally insert additional variables)
  // USER END

  switch (pMsg->MsgId) {
  case WM_PAINT:
	  GUI_DrawGradientV(0, 0, 799, 439, GUI_WHITE, GUI_CYAN);
	  GUI_SetTextMode(GUI_TM_TRANS);
	  GUI_SetColor(GUI_BLACK);
	  GUI_SetFont(&GUI_Font20B_1);
	  GUI_DispStringAt("POS:",580,320);
	  GUI_DispDecAt(Select,630,320,1);
	  GUI_DispStringAt("SEL:",580,290);
	  GUI_DispDecAt(Select2, 630, 290, 1);
	  break;
  case WM_INIT_DIALOG:
    //
    // Initialization of 'Listwheel'
    //
    hItem = WM_GetDialogItem(pMsg->hWin, ID_LISTWHEEL_0);
	LISTWHEEL_SetLineHeight(hItem, 60);
	LISTWHEEL_SetBkColor(hItem, LISTWHEEL_CI_UNSEL, GUI_WHITE);
	LISTWHEEL_SetTextAlign(hItem, GUI_TA_VCENTER);



	LISTWHEEL_SetFont(hItem, &GUI_Font32B_1);
    LISTWHEEL_AddString(hItem, "ZERO");
    LISTWHEEL_AddString(hItem, "ONE");
    LISTWHEEL_AddString(hItem, "TWO");
    LISTWHEEL_AddString(hItem, "THREE");
	LISTWHEEL_AddString(hItem, "FOUR");
	LISTWHEEL_AddString(hItem, "FIVE");
	LISTWHEEL_AddString(hItem, "SIX");
	LISTWHEEL_AddString(hItem, "SEVEN");
	LISTWHEEL_SetOwnerDraw(hItem, _OwnerDraw);

	CALENDAR_SetDefaultFont(CALENDAR_FI_CONTENT,&GUI_Font32B_1);
	CALENDAR_SetDefaultFont(CALENDAR_FI_HEADER, &GUI_Font32_1);

	CALENDAR_SetDefaultSize(CALENDAR_SI_HEADER,40);
	CALENDAR_SetDefaultSize(CALENDAR_SI_CELL_X,80);
	CALENDAR_SetDefaultSize(CALENDAR_SI_CELL_Y,50);

	hCalen = CALENDAR_Create(pMsg->hWin,20,20,Date.Year,Date.Month,Date.Day,0,0,0);
    // USER START (Optionally insert additional code for further widget initialization)
    // USER END
    break;
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);
    NCode = pMsg->Data.v;
    switch(Id) {
    case ID_BUTTON_0: // Notifications sent by 'return'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
		  WM_DeleteWindow(pMsg->hWin);
		  CreateWin1();
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_BUTTON_1: // Notifications sent by 'OK'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
		  hItem = WM_GetDialogItem(pMsg->hWin, ID_LISTWHEEL_0);
		  Select2 = LISTWHEEL_GetPos(hItem);

		  if (Select2 >5)Select2 -= 6;
		  else Select2 += 2;
		  LISTWHEEL_SetSel(hItem,Select2);
		  Select = Select2;
		  // hCalen
		  CALENDAR_GetSel(hCalen, &Date);
		  CALENDAR_SetDate(hCalen, &Date);

		  WM_SendMessageNoPara(WIN_Header, DATE_DIS);
		  WM_InvalidateWindow(pMsg->hWin);
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    case ID_LISTWHEEL_0: // Notifications sent by 'Listwheel'
      switch(NCode) {
      case WM_NOTIFICATION_CLICKED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_RELEASED:
        // USER START (Optionally insert code for reacting on notification message)
        // USER END
        break;
      case WM_NOTIFICATION_SEL_CHANGED:
        // USER START (Optionally insert code for reacting on notification message)
		  hItem = WM_GetDialogItem(pMsg->hWin, ID_LISTWHEEL_0);
		  Select = LISTWHEEL_GetPos(hItem);
		  Select2 = LISTWHEEL_GetSel(hItem);
		  WM_InvalidateWindow(pMsg->hWin);
        // USER END
        break;
      // USER START (Optionally insert additional code for further notification handling)
      // USER END
      }
      break;
    // USER START (Optionally insert additional code for further Ids)
    // USER END
    }
    break;
  // USER START (Optionally insert additional message handling)
  // USER END
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       CreateWindow
*/
WM_HWIN CreateWin3(void);
WM_HWIN CreateWin3(void) {
  WM_HWIN hWin;

  hWin = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), _cbDialog, WM_HBKWIN, 0, 0);
  return hWin;
}

// USER START (Optionally insert additional public code)
// USER END

/*************************** End of file ****************************/
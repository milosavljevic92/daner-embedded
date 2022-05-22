VERSION 5.00
Begin VB.Form Form1 
   BorderStyle     =   1  'Fixed Single
   Caption         =   "Prevodilac"
   ClientHeight    =   8910
   ClientLeft      =   45
   ClientTop       =   375
   ClientWidth     =   7440
   LinkTopic       =   "Form1"
   MaxButton       =   0   'False
   MinButton       =   0   'False
   ScaleHeight     =   8910
   ScaleWidth      =   7440
   StartUpPosition =   2  'CenterScreen
   Begin VB.CommandButton Command3 
      Caption         =   "Kreni"
      Height          =   615
      Left            =   1680
      TabIndex        =   5
      Top             =   960
      Width           =   1815
   End
   Begin VB.CommandButton Command2 
      Caption         =   "-"
      Height          =   495
      Left            =   6360
      TabIndex        =   4
      Top             =   1320
      Width           =   735
   End
   Begin VB.TextBox kraj 
      Height          =   6615
      Left            =   120
      MultiLine       =   -1  'True
      TabIndex        =   3
      Text            =   "FrmMain.frx":0000
      Top             =   1920
      Width           =   5775
   End
   Begin VB.CommandButton Command1 
      Caption         =   "Unesi"
      Height          =   495
      Left            =   4320
      TabIndex        =   2
      Top             =   1320
      Width           =   1575
   End
   Begin VB.Timer Trm 
      Interval        =   50
      Left            =   3960
      Top             =   120
   End
   Begin VB.TextBox TxtPrviRed 
      BeginProperty Font 
         Name            =   "Arial"
         Size            =   12
         Charset         =   238
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   375
      Left            =   240
      MaxLength       =   16
      TabIndex        =   0
      Top             =   120
      Width           =   5655
   End
   Begin VB.Label LblCount 
      Caption         =   "16"
      BeginProperty Font 
         Name            =   "Arial"
         Size            =   12
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   255
      Left            =   240
      TabIndex        =   1
      Top             =   1080
      Width           =   495
   End
End
Attribute VB_Name = "Form1"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
Private Sub Command1_Click()
Dim sFileText As String
Dim iFileNo As Integer
  iFileNo = FreeFile
  Open "C:\izlaz.txt" For Output As #iFileNo

 


 
      'close the file (if you dont do this, you wont be able to open it again!)
  Close #iFileNo
End Sub

Private Sub Command2_Click()
kraj.Text = kraj.Text & vbNewLine & "Damir"
End Sub

Private Sub Command3_Click()
Dim sFileText As String
Dim iFileNo As Integer
  iFileNo = FreeFile
      'open the file for reading
  Open "C:\ulaz.txt" For Input As #iFileNo

 
      'read the file until we reach the end
  Do While Not EOF(iFileNo)
    Input #iFileNo, sFileText
    sfil
      'show the text (you will probably want to replace this line as appropriate to your program!)
    MsgBox sFileText
  Loop
 
      'close the file (if you dont do this, you wont be able to open it again!)
  Close #iFileNo
End Sub

Private Sub Form_Load()

End Sub

Private Sub Trm_Timer()
LblCount.Caption = Len(TxtPrviRed.Text)
End Sub

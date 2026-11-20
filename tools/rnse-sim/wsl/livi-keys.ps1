# Types keys into LIVI's WSLg window, one Windows virtual-key code per stdin
# line, and gives focus back. Keys go with their scan codes: WSLg's RDP
# client forwards scan codes, so SendKeys-style input without them is lost.
# rnse-sim starts it (WSL interop) for the keys carpid sends
# to LIVI's menus: under WSLg those come from Windows, not uinput.
Add-Type @"
using System;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.RegularExpressions;
public static class Win {
    public delegate bool Proc(IntPtr h, IntPtr l);
    [DllImport("user32.dll")] static extern bool EnumWindows(Proc f, IntPtr l);
    [DllImport("user32.dll")] static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
    [DllImport("user32.dll")] static extern bool IsWindowVisible(IntPtr h);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
    [DllImport("user32.dll")] public static extern IntPtr GetForegroundWindow();
    [DllImport("user32.dll")] public static extern void keybd_event(byte vk, byte scan, uint flags, UIntPtr extra);
    [DllImport("user32.dll")] static extern uint MapVirtualKey(uint code, uint type);
    public static void Key(byte vk) {
        byte scan = (byte)MapVirtualKey(vk, 0);
        // Arrows, Page Up/Down, Home/End, Insert/Delete are extended keys.
        uint ext = (vk >= 0x21 && vk <= 0x28) || vk == 0x2D || vk == 0x2E ? 1u : 0u;
        keybd_event(vk, scan, ext, UIntPtr.Zero);
        System.Threading.Thread.Sleep(20);
        keybd_event(vk, scan, ext | 2u, UIntPtr.Zero);
    }
    public static IntPtr Find(string pattern) {
        IntPtr found = IntPtr.Zero;
        EnumWindows((h, l) => {
            if (!IsWindowVisible(h)) return true;
            var sb = new StringBuilder(256);
            GetWindowText(h, sb, 256);
            if (Regex.IsMatch(sb.ToString(), pattern)) { found = h; return false; }
            return true;
        }, IntPtr.Zero);
        return found;
    }
    public static void Focus(IntPtr h) {
        if (!SetForegroundWindow(h)) {
            keybd_event(0x12, 0, 0, UIntPtr.Zero);
            keybd_event(0x12, 0, 2, UIntPtr.Zero);
            SetForegroundWindow(h);
        }
    }
}
"@
while ($null -ne ($k = [Console]::In.ReadLine())) {
    # WSLg titles look like "LIVI (NixOS)", sometimes with a "[...] " prefix.
    $h = [Win]::Find('^(\[[^\]]*\] )?LIVI \(')
    if ($h -eq [IntPtr]::Zero) { continue }
    $cur = [Win]::GetForegroundWindow()
    if ($cur -ne $h) { [Win]::Focus($h); Start-Sleep -Milliseconds 30 }
    [Win]::Key([byte][int]$k)
}

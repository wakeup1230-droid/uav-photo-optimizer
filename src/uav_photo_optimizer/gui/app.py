"""
Simple GUI (tkinter). Six fields + 開始篩選, and a completion dialog. Nothing else.

No algorithm here: the form is sent to the REST API v1 (``gui/client.py``).
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from ..core.config import (DEFAULT_BUFFER_M, DEFAULT_FRONT_OVERLAP, DEFAULT_SIDE_OVERLAP,
                           MAX_OVERLAP, MIN_BUFFER_M, MIN_OVERLAP)

TITLE = "UAV 照片篩選工具"
POLL_MS = 500
STAGES = {"scan": "搜尋照片…", "metadata": "讀取照片資訊…", "geometry": "建立照片幾何…",
          "optimize": "計算重疊與減量…", "copy": "複製照片…", "done": "完成"}


class FormError(ValueError):
    pass


def build_request(aoi: str, photos: str, buffer: str, front: str, side: str,
                  output: str) -> dict:
    """Validate the form (same limits as Core / API) → API v1 request body."""
    aoi, photos, output = aoi.strip(), photos.strip(), output.strip()
    if not aoi:
        raise FormError("請選擇建模範圍（.shp）")
    if Path(aoi).suffix.lower() != ".shp":
        raise FormError("建模範圍必須是 .shp 檔")
    if not photos:
        raise FormError("請選擇照片資料夾")
    if not output:
        raise FormError("請選擇輸出資料夾")

    def number(text: str, name: str) -> float:
        try:
            return float(text.strip())
        except ValueError:
            raise FormError(f"{name}必須是數字") from None

    b = number(buffer, "Buffer")
    f = number(front, "航向重疊率")
    s = number(side, "側向重疊率")
    if b < MIN_BUFFER_M:
        raise FormError(f"Buffer 不可小於 {MIN_BUFFER_M:g} m")
    for v, name in ((f, "航向重疊率"), (s, "側向重疊率")):
        if not MIN_OVERLAP <= v <= MAX_OVERLAP:
            raise FormError(f"{name}需介於 {MIN_OVERLAP:g}～{MAX_OVERLAP:g}%")
    return {"aoi_shapefile": aoi, "photo_dir": photos, "output_dir": output, "buffer_m": b,
            "front_overlap": f, "side_overlap": s, "copy_photos": True}


def completion_text(result: dict) -> str:
    lines = [f"候選照片：{result['candidate_photos']}",
             f"保留照片：{result['selected_photos']}",
             f"減少照片：{result['removed_photos']}",
             f"減量比例：{result['reduction_percent']:.1f}%"]
    if result.get("original_overlap_below_target"):
        lines += ["", "部分原始照片重疊率低於設定值。"]
    return "\n".join(lines)


class SelectionWindow:
    def __init__(self, root, client):
        import tkinter as tk
        from tkinter import ttk

        self.root, self.client = root, client
        self.job_id: Optional[str] = None
        root.title(TITLE)
        root.resizable(False, False)
        frm = ttk.Frame(root, padding=16)
        frm.grid()
        ttk.Label(frm, text=TITLE, font=("", 14, "bold")).grid(row=0, column=0, columnspan=3,
                                                                 pady=(0, 12), sticky="w")
        self.aoi, self.photos, self.output = tk.StringVar(), tk.StringVar(), tk.StringVar()
        self.buffer = tk.StringVar(value=f"{DEFAULT_BUFFER_M:g}")
        self.front = tk.StringVar(value=f"{DEFAULT_FRONT_OVERLAP:g}")
        self.side = tk.StringVar(value=f"{DEFAULT_SIDE_OVERLAP:g}")
        self.status = tk.StringVar()

        def path_row(r, label, var, pick):
            ttk.Label(frm, text=label).grid(row=r, column=0, sticky="w", pady=3)
            ttk.Entry(frm, textvariable=var, width=48).grid(row=r, column=1, pady=3)
            ttk.Button(frm, text="...", width=3, command=pick).grid(row=r, column=2, padx=(4, 0))

        def num_row(r, label, var, unit):
            ttk.Label(frm, text=label).grid(row=r, column=0, sticky="w", pady=3)
            box = ttk.Frame(frm)
            box.grid(row=r, column=1, sticky="w")
            ttk.Entry(box, textvariable=var, width=8, justify="right").pack(side="left")
            ttk.Label(box, text=f" {unit}").pack(side="left")

        path_row(1, "建模範圍", self.aoi, self._pick_aoi)
        path_row(2, "照片資料夾", self.photos, lambda: self._pick_dir(self.photos))
        num_row(3, "Buffer", self.buffer, "m")
        num_row(4, "航向重疊率", self.front, "%")
        num_row(5, "側向重疊率", self.side, "%")
        path_row(6, "輸出資料夾", self.output, lambda: self._pick_dir(self.output))
        self.start_btn = ttk.Button(frm, text="開始篩選", command=self.start)
        self.start_btn.grid(row=7, column=0, columnspan=3, pady=(14, 4))
        ttk.Label(frm, textvariable=self.status, foreground="gray").grid(row=8, column=0,
                                                                         columnspan=3)

    # -- pickers ---------------------------------------------------------------------------

    def _pick_aoi(self):
        from tkinter import filedialog
        p = filedialog.askopenfilename(title="建模範圍",
                                       filetypes=[("Shapefile", "*.shp"), ("All", "*.*")])
        if p:
            self.aoi.set(p)

    def _pick_dir(self, var):
        from tkinter import filedialog
        p = filedialog.askdirectory(title="選擇資料夾")
        if p:
            var.set(p)

    # -- run -------------------------------------------------------------------------------

    def start(self):
        from tkinter import messagebox

        from .client import ApiError
        try:
            body = build_request(self.aoi.get(), self.photos.get(), self.buffer.get(),
                                 self.front.get(), self.side.get(), self.output.get())
            self.job_id = self.client.create_job(body)["job_id"]
        except FormError as exc:
            messagebox.showwarning(TITLE, str(exc))
            return
        except ApiError as exc:
            messagebox.showerror(TITLE, exc.message)
            return
        self.start_btn.state(["disabled"])
        self.status.set("處理中…")
        self.root.after(POLL_MS, self.poll)

    def poll(self):
        from tkinter import messagebox

        from .client import ApiError
        try:
            job = self.client.job(self.job_id)
            if job["status"] in ("queued", "running"):
                p = job["progress"]
                text = STAGES.get(p.get("stage") or "", "處理中…")
                if p.get("total"):
                    text += f"  {p['done']} / {p['total']}"
                self.status.set(text)
                self.root.after(POLL_MS, self.poll)
                return
            self.start_btn.state(["!disabled"])
            self.status.set("")
            if job["status"] == "succeeded":
                messagebox.showinfo("篩選完成", completion_text(self.client.result(self.job_id)))
            else:
                messagebox.showerror(TITLE, (job.get("error") or {}).get("message", "篩選失敗"))
        except ApiError as exc:
            self.start_btn.state(["!disabled"])
            self.status.set("")
            messagebox.showerror(TITLE, exc.message)


def main() -> int:
    import tkinter as tk
    from tkinter import messagebox

    from .client import connect
    root = tk.Tk()
    try:
        client, server = connect()
    except ImportError:
        root.withdraw()
        messagebox.showerror(TITLE, "GUI 需要 REST API 套件：pip install "
                                    "\"uav-photo-optimizer[gui]\"")
        return 1
    SelectionWindow(root, client)
    try:
        root.mainloop()
    finally:
        if server is not None:
            server.stop()
    return 0

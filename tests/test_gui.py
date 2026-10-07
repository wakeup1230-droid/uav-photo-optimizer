"""Simple GUI: form validation, completion dialog, API client + local server, window smoke."""

import time
from pathlib import Path

import pytest

from uav_photo_optimizer.gui.app import FormError, build_request, completion_text

FORM = dict(aoi="C:/data/area.shp", photos="C:/data/photos", buffer="100", front="80",
            side="70", output="C:/data/out")


def test_build_request_defaults_and_limits():
    body = build_request(**FORM)
    assert body == {"aoi_shapefile": "C:/data/area.shp", "photo_dir": "C:/data/photos",
                    "output_dir": "C:/data/out", "buffer_m": 100.0, "front_overlap": 80.0,
                    "side_overlap": 70.0, "copy_photos": True}
    for bad, msg in [({"front": "64"}, "航向重疊率"), ({"side": "96"}, "側向重疊率"),
                     ({"buffer": "-1"}, "Buffer"), ({"front": "abc"}, "數字"),
                     ({"aoi": ""}, "建模範圍"), ({"aoi": "x.txt"}, ".shp"),
                     ({"photos": ""}, "照片資料夾"), ({"output": " "}, "輸出資料夾")]:
        with pytest.raises(FormError, match=msg):
            build_request(**{**FORM, **bad})


def test_completion_text():
    r = {"candidate_photos": 1200, "selected_photos": 860, "removed_photos": 340,
         "reduction_percent": 28.33, "original_overlap_below_target": False}
    assert completion_text(r) == ("候選照片：1200\n保留照片：860\n減少照片：340\n"
                                  "減量比例：28.3%")
    assert completion_text({**r, "original_overlap_below_target": True}).endswith(
        "部分原始照片重疊率低於設定值。")


def test_client_against_local_server(dataset):
    pytest.importorskip("uvicorn")
    from uav_photo_optimizer.gui.client import ApiClient, ApiError, LocalServer
    base, shp, photo_dir = dataset
    server = LocalServer(base_dir=base).start()
    try:
        c = ApiClient(server.url)
        assert c.health()["status"] == "ok"
        body = build_request(str(shp), str(photo_dir), "0", "65", "65", str(base / "gui_out"))
        job = c.create_job(body)
        for _ in range(600):
            st = c.job(job["job_id"])["status"]
            if st in ("succeeded", "failed"):
                break
            time.sleep(0.05)
        assert st == "succeeded"
        res = c.result(job["job_id"])
        assert res["copied_photos"] == res["selected_photos"] > 0
        assert Path(res["output_dir"]).parent == base / "gui_out"
        with pytest.raises(ApiError) as e:
            c.create_job({**body, "aoi_shapefile": str(base / "none.shp")})
        assert e.value.status == 400
    finally:
        server.stop()


def test_window_has_only_the_six_fields():
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("no display")
    try:
        from uav_photo_optimizer.gui.app import SelectionWindow
        w = SelectionWindow(root, client=None)
        labels = []

        def walk(widget):
            for child in widget.winfo_children():
                if child.winfo_class() in ("TLabel", "TButton"):
                    labels.append(child.cget("text"))
                walk(child)

        walk(root)
        texts = [t for t in labels if t not in ("...", "", " m", " %")]
        assert texts == ["UAV 照片篩選工具", "建模範圍", "照片資料夾", "Buffer", "航向重疊率",
                         "側向重疊率", "輸出資料夾", "開始篩選"]
        assert (w.buffer.get(), w.front.get(), w.side.get()) == ("100", "80", "70")
    finally:
        root.destroy()

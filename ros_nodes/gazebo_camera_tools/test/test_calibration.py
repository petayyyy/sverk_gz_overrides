from array import array
from types import SimpleNamespace
from gazebo_camera_tools.calibration import to_opencv


def test_corrects_k_and_projection_without_changing_other_fields_or_source():
    source = SimpleNamespace(k=array('d', [871, 0, 597.8472, 0, 872, 519.0569, 0, 0, 1]),
        p=array('d', [871, 0, 597.8472, -87, 0, 872, 519.0569, 0, 0, 0, 1, 0]),
        header={'stamp':123, 'frame_id':'optical'}, d=[0]*5, width=1224, height=1024,
        binning_x=2, roi={'x_offset':10})
    result = to_opencv(source)
    assert result.k[2] == 597.3472 and result.k[5] == 518.5569
    assert result.p[2] == result.k[2] and result.p[6] == result.k[5]
    assert result.p[3] == -87
    assert source.k[2] == 597.8472
    for key in ('header','d','width','height','binning_x','roi'):
        assert getattr(result,key) == getattr(source,key)


def test_uncalibrated_camera_info_stays_uncalibrated():
    source = SimpleNamespace(k=[0.]*9,p=[0.]*12)
    assert to_opencv(source) == source

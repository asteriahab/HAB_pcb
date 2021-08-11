import pytest
from wind_tunnel2 import temp_init, read_temp, clear_adt, get_adt

# Dummy exception for mocking
EXCEPT = Exception('mocked')

@pytest.fixture(autouse=True)
def mock_temp_obj(mocker):
    return mocker.patch('adafruit_adt7410.ADT7410')

@pytest.fixture(autouse=True)
def mock_busio(mocker):
    return mocker.patch('busio.I2C')

@pytest.fixture(autouse=True)
def temp_failed_value(self):
    return None
#=================================================================================================
# TEST temp_init()
#=================================================================================================
class TestTempInit:
    @pytest.fixture
    def temp_init_success_setup(self):
        clear_adt()

        yield

        clear_adt()

    @pytest.fixture
    def temp_init_exception_setup(self):
        temp_init()

        yield

        clear_adt()
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def test_temp_init_success(self, temp_init_success_setup):
        temp_init()

        temp_init_result = get_adt()

        assert temp_init_result != temp_failed_value

    def test_temp_init_busio_exception(self, temp_init_exception_setup):
        mocker.patch('busio.I2C', side_effect = EXCEPT)
        temp_init()

        temp_init_result = get_adt()

        assert temp_init_result == temp_failed_value

    def test_temp_init_adt_exception(self, temp_init_exception_setup):
        mocker.patch('adafruit_adt7410.ADT7410', side_effect = EXCEPT)
        temp_init()

        temp_init_result = get_adt()

        assert adt_exception_adt == temp_failed_value

#=================================================================================================
# TEST read_temp()
#=================================================================================================
class TestReadTemp:
    def test_read_temp_success(self, good_adt):
        temp_init()

        read_temp_result = read_temp()

        assert good_adt != temp_failed_value

    def test_read_temp_exception(self):
        clear_adt()
        mocker.patch('wind_tunnel2.temp_init()')

        read_temp_result = read_temp()

        assert read_temp_result == temp_failed_value

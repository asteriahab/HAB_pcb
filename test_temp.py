import pytest
from wind_tunnel2 import temp_init, read_temp, clear_adt, get_adt

# Dummy exception for mocking
EXCEPT = Exception('mocked')

@pytest.fixture()
def mock_temp_obj(mocker):
    return mocker.patch('adafruit_adt7410.ADT7410')

@pytest.fixture()
def mock_busio(mocker):
    return mocker.patch('busio.I2C')
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
    def temp_init_exception_setup(self, mock_busio, mock_temp_obj):
        temp_init()

        yield

        clear_adt()
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def test_temp_init_success(self, temp_init_success_setup, mock_busio, mock_temp_obj):
        temp_init()

        result = get_adt()

        assert result is not None

    def test_temp_init_busio_exception(self, temp_init_exception_setup, mock_busio, mock_temp_obj):
        mock_busio.side_effect = EXCEPT
        temp_init()

        result = get_adt()

        assert result is None

    def test_temp_init_adt_exception(self, temp_init_exception_setup, mock_busio, mock_temp_obj):
        mock_temp_obj.side_effect = EXCEPT
        temp_init()

        result = get_adt()

        assert result is None

#=================================================================================================
# TEST read_temp()
#=================================================================================================
class TestReadTemp:
    def test_read_temp_success(self, mock_busio, mock_temp_obj):
        temp_init()

        result = read_temp()

        assert result is not None

    def test_read_temp_exception(self, mocker):
        clear_adt()
        mocker.patch('wind_tunnel2.temp_init')

        result = read_temp()

        assert result is None

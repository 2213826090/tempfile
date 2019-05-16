# -*- coding: utf-8 -*-

from testlib.graphics.display_size_impl import DisplaySizeImpl


class DisplaySize(DisplaySizeImpl):

    _ALLSize = ['Default','Large','Larger','Largest','Small']

    def setUp(self):
        self.set_displaysize_to_default()

    def tearDown(self):
        self.set_displaysize_to_default()

    def test_Screen_UI_Zoom_Default(self):
        self.launch_display_size_menu()
        self.change_display_size_and_check('Default')

    def test_Screen_UI_Zoom_Large(self):
        self.launch_display_size_menu()
        self.change_display_size_and_check('Large')

    def test_Screen_UI_Zoom_Larger(self):
        self.launch_display_size_menu()
        self.change_display_size_and_check('Larger')

    def test_Screen_UI_Zoom_Largest(self):
        self.launch_display_size_menu()
        self.change_display_size_and_check('Largest')

    def test_Screen_UI_Zoom_Small(self):
        self.launch_display_size_menu()
        self.change_display_size_and_check('Small')

    def test_Screen_UI_Zoom_switch_repeatedly(self):
        self.launch_display_size_menu()
        for ds in self._ALLSize:
            self.change_display_size_and_check(ds)

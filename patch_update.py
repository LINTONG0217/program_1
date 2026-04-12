import sys

def patch_run_forever():
    with open('c:\\Users\\LINTONG\\Desktop\\car\\program_1\\program_1\\Module\\task_controller_basic.py', 'r', encoding='utf-8') as f:
        src = f.read()
    
    rep = '''        def run_forever(self):
                while True:
                        try:
                                self.update()
                        except Exception as e:
                                import sys
                                print(" CRASH IN UPDATE!!!\)
 sys.print_exception(e)
 raise
 time.sleep_ms(self.cfg.LOOP_DELAY_MS)'''
 
 src = src.replace(' def run_forever(self):\n while True:\n self.update()\n time.sleep_ms(self.cfg.LOOP_DELAY_MS)', rep)
 
 with open('c:\\Users\\LINTONG\\Desktop\\car\\program_1\\program_1\\Module\\task_controller_basic.py', 'w', encoding='utf-8') as f:
 f.write(src)

patch_run_forever()

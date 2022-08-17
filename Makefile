test.out: pid_tester.cpp pid_controller.cpp pid_controller.h
	@ g++ pid_controller.h pid_controller.cpp pid_tester.cpp -o test.out
	
test: test.out
	-@ rm test_results/* 2>/dev/null || true #Do not generate error if nothing is in directory
	@ ./test.out
    
clean:
	-@ rm test_results/* 2>/dev/null || true #Do not generate error if nothing is in directory
	-@ rm test.out 2>/dev/null || true #Do not generate error if no executable present

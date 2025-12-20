USER_DIR=ARGoS_based_Air_Resistance___User_Manual
DEV_DIR=ARGoS_based_Air_Resistance___developer_manual
OUT_DIR=pdf

.PHONY: docs user developer clean

docs: user developer

user:
	cd $(USER_DIR) && latexmk -pdf -interaction=nonstopmode -halt-on-error main.tex
	mkdir -p $(OUT_DIR)
	cp $(USER_DIR)/main.pdf $(OUT_DIR)/ARGoS_based_Air_Resistance__User_Manual.pdf

developer:
	cd $(DEV_DIR) && latexmk -pdf -interaction=nonstopmode -halt-on-error main.tex
	mkdir -p $(OUT_DIR)
	cp $(DEV_DIR)/main.pdf $(OUT_DIR)/ARGoS_based_Air_Resistance__developer_manual.pdf

clean:
	cd $(USER_DIR) && latexmk -C
	cd $(DEV_DIR) && latexmk -C
	rm -rf $(OUT_DIR)

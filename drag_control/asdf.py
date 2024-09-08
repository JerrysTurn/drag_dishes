class Config:
    def __init__(self, config=None):
        # 딕셔너리로 기본값 설정
        default_config = {
            'batch_size': 32,
            'learning_rate': 0.001,
            'epochs': 10,
            'model_name': "base_model"
        }
        # 사용자가 전달한 설정으로 덮어쓰기
        if config:
            default_config.update(config)
        
        # 각 변수에 설정값 할당
        self.batch_size = default_config['batch_size']
        self.learning_rate = default_config['learning_rate']
        self.epochs = default_config['epochs']
        self.model_name = default_config['model_name']
    
    def print_config(self):
        print(f"Batch Size: {self.batch_size}")
        print(f"Learning Rate: {self.learning_rate}")
        print(f"Epochs: {self.epochs}")
        print(f"Model Name: {self.model_name}")

# 딕셔너리로 값을 전달하여 설정 변경
custom_config = {'batch_size': 64, 'learning_rate': 0.0005, 'model_name': 'custom_model'}
config = Config(config=custom_config)
config.print_config()

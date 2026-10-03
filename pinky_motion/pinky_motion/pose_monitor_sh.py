"""관제 PC 위치 모니터 실행 파일."""

# 실제 구독 로직은 공통 모듈에서 재사용한다.
from .pose_monitor import main


if __name__ == '__main__':
    main()

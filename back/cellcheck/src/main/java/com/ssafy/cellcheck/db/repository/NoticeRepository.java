package com.ssafy.cellcheck.db.repository;

import com.ssafy.cellcheck.db.entity.Notice;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import java.util.List;

@Repository
public interface NoticeRepository extends JpaRepository<Notice, Long> {

    List<Notice> findByUserUserIdOrderByCreatedAtDesc(Long userId);
}